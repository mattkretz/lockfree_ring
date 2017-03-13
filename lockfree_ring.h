/*{{{
Copyright © 2017 Matthias Kretz <kretz@kde.org>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the names of contributing organizations nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

}}}*/

#ifndef VIR_LOCKFREE_RING_H_
#define VIR_LOCKFREE_RING_H_

#include <array>
#include <atomic>
#include <cassert>
#ifdef __has_include
#  if __has_include(<optional>)
#    include <optional>
namespace vir { using std::optional; }
#  elif __has_include(<experimental/optional>)
#    include <experimental/optional>
namespace vir { using std::experimental::optional; }
#  elif __has_include(<boost/optional.hpp>)
#    include <boost/version.hpp>
#    if BOOST_VERSION < 105600
#      error "Boost >= 1.56 is required for proper move semantics"
#    endif
#    include <boost/optional.hpp>
namespace vir { using boost::optional; }
#  else
#    error "optional<T> not found. Use C++17, a compiler implementing the library fundamentals TS, or Boost.Optional."
#  endif
#else
#  include <boost/optional.hpp>
namespace vir { using boost::optional; }
#endif
#include <type_traits>
#include <utility>

#define VIR_NODISCARD
#if defined __has_cpp_attribute
#if __has_cpp_attribute(nodiscard)
#undef VIR_NODISCARD
#define VIR_NODISCARD [[nodiscard]]
#endif
#endif

namespace vir {
namespace detail
{
template <class T> static constexpr T msb() { return ~T() ^ (~T() >> 1); }
}  // namespace detail
template <class T, size_t N, bool = 0u == (N & (N - 1u)) &&      // require power of 2
                                    (N < detail::msb<size_t>())  // require MSB(N) == 0
          >
class lockfree_ring;
template <class T, size_t N> class lockfree_ring<T, N, true>
{
  static constexpr bool nothrow_movable_T = std::is_nothrow_move_constructible<T>::value;
  static constexpr size_t index_mask = N - 1;
  enum class ReadyState : unsigned char { Empty, Writing, Ready, Reading };

  // avoid false sharing via alignment to the typical cacheline size
  struct alignas(64) Bucket {
    typename std::aligned_storage<sizeof(T), alignof(T)>::type object_storage;
    std::atomic<ReadyState> ready;

    Bucket() noexcept : ready(ReadyState::Empty) {}

    Bucket(Bucket &&rhs) noexcept(nothrow_movable_T) : ready(rhs.ready.load())
    {
      ReadyState state = ready;
      if (state == ReadyState::Ready) {
        new (&object_storage) T(std::move(reinterpret_cast<T &&>(rhs.object_storage)));
      } else {
        // if state is Writing or Reading there's a race on move => UB!
        assert(state == ReadyState::Empty);
      }
      rhs.ready = ReadyState::Empty;
    }

    ~Bucket()
    {
      ReadyState state = ready;
      if (state == ReadyState::Ready) {
        T &obj = reinterpret_cast<T &>(object_storage);
        obj.~T();
      } else {
        // if state is Writing or Reading there's a race on destruction => UB!
        assert(state == ReadyState::Empty);
      }
    }
  };

public:
  template <class U> class writer
  {
    friend lockfree_ring;
    Bucket &bucket;
    U &&payload;
    bool did_write = false;
    writer(Bucket &b, U &&p) noexcept : bucket(b), payload(std::forward<U>(p)) {}

  public:
    writer(writer &&rhs) noexcept : bucket(rhs.bucket),
                                    payload(std::forward<U>(rhs.payload)),
                                    did_write(rhs.did_write)
    {
      rhs.did_write = true;
    }

    VIR_NODISCARD bool try_push() noexcept(noexcept(T(std::declval<U>())))
    {
      assert(!did_write);
      ReadyState expected = ReadyState::Empty;
      if (bucket.ready.compare_exchange_weak(expected, ReadyState::Writing)) {
        new (&bucket.object_storage) T(std::forward<U>(payload));
        bucket.ready = ReadyState::Ready;
        did_write = true;
        return true;
      }
      return false;
    }

    inline ~writer() { assert(did_write); }
  };

  class reader {
    friend lockfree_ring;
    Bucket &b;
    bool did_read = false;
    reader(Bucket &bb) : b(bb) {}

  public:
    reader(reader &&rhs) noexcept : b(rhs.b), did_read(rhs.did_read)
    {
      rhs.did_read = true;
    }

    VIR_NODISCARD optional<T> get() noexcept(nothrow_movable_T)
    {
      assert(!did_read);
      ReadyState expected = ReadyState::Ready;
      if (b.ready.compare_exchange_weak(expected, ReadyState::Reading)) {
        T &stored = reinterpret_cast<T &>(b.object_storage);
        optional<T> ret(std::move(stored));
        stored.~T();
        b.ready = ReadyState::Empty;
        did_read = true;
        return ret;
      }
      return {};
    }

    inline ~reader() { assert(did_read); }
  };

  lockfree_ring() noexcept : write_index(0u), read_index(0u) {}

  lockfree_ring(lockfree_ring &&rhs) noexcept(
      std::is_nothrow_move_constructible<Bucket>::value)
      : write_index(rhs.write_index.load())
      , read_index(rhs.read_index.load())
      , buffer(std::move(rhs.buffer))
  {
  }

  template <class U>
  VIR_NODISCARD
      typename std::enable_if<std::is_constructible<T, U &&>::value, writer<U>>::type
      prepare_push(U &&x) noexcept
  {
      const auto wi = write_index.fetch_add(1u) & index_mask;
      return {buffer[wi], std::forward<U>(x)};
  }

  VIR_NODISCARD reader pop_front() noexcept
  {
    const auto ri = read_index.fetch_add(1u) & index_mask;
    return {buffer[ri]};
  }

private:
  std::atomic<size_t> write_index;
  std::atomic<size_t> read_index;
  std::array<Bucket, N> buffer;
};
}  // namespace vir

#undef VIR_NODISCARD
#endif  // VIR_LOCKFREE_RING_H_
