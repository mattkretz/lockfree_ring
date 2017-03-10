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
#include <experimental/optional>
#include <type_traits>
#include <utility>

#if defined __has_cpp_attribute && __has_cpp_attribute(nodiscard)
#define VIR_NODISCARD [[nodiscard]]
#else
#define VIR_NODISCARD
#endif

namespace vir {
namespace detail
{
template <class T> static constexpr T msb = ~T() ^ (~T() >> 1);
}  // namespace detail
template <class T, size_t N, bool = 0u == (N & (N - 1u)) &&    // require power of 2
                                    (N < detail::msb<size_t>)  // require MSB(N) == 0
          >
class lockfree_ring;
template <class T, size_t N> class lockfree_ring<T, N, true>
{
  static constexpr size_t index_mask = N - 1;
  enum class ReadyState : unsigned char { Empty, Writing, Ready, Reading };

  // avoid false sharing via alignment to the typical cacheline size
  struct alignas(64) Bucket {
    typename std::aligned_storage<sizeof(T), alignof(T)>::type object_storage;
    std::atomic<ReadyState> ready{ReadyState::Empty};

    Bucket() = default;

    Bucket(Bucket &&rhs) : ready(std::move(rhs.ready)) {
      ReadyState state = ready;
      if (state == ReadyState::Ready) {
        new (&object_storage) T(std::move(reinterpret_cast<T &&>(rhs.object_storage)));
      } else {
        // if state is Writing or Reading there's a race on move!
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
        // if state is Writing or Reading there's a race on destruction!
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
    writer(Bucket &b, U &&p) : bucket(b), payload(std::forward<U>(p)) {}

  public:
    writer(writer &&rhs)
        : bucket(rhs.bucket)
        , payload(std::forward<U>(rhs.payload))
        , did_write(rhs.did_write)
    {
      rhs.did_write = true;
    }

    VIR_NODISCARD bool try_push()
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
    reader(reader &&rhs) : b(rhs.b), did_read(rhs.did_read) { rhs.did_read = true; }

    VIR_NODISCARD std::experimental::optional<T> get()
    {
      assert(!did_read);
      ReadyState expected = ReadyState::Ready;
      if (b.ready.compare_exchange_weak(expected, ReadyState::Reading)) {
        T &stored = reinterpret_cast<T &>(b.object_storage);
        std::experimental::optional<T> ret(std::move(stored));
        stored.~T();
        b.ready = ReadyState::Empty;
        did_read = true;
        return ret;
      }
      return {};
    }

    inline ~reader() { assert(did_read); }
  };

  lockfree_ring() = default;

  lockfree_ring(lockfree_ring &&rhs) = default;

  template <class U>
  VIR_NODISCARD std::enable_if_t<std::is_constructible<T, U &&>::value, writer<U>>
  prepare_push(U &&x)
  {
      const auto wi = write_index.fetch_add(1u) & index_mask;
      return {buffer[wi], std::forward<U>(x)};
  }

  VIR_NODISCARD reader pop_front()
  {
    const auto ri = read_index.fetch_add(1u) & index_mask;
    return {buffer[ri]};
  }

private:
  std::atomic<size_t> write_index{0u};
  std::atomic<size_t> read_index{0u};
  std::array<Bucket, N> buffer;
};
}  // namespace vir

#undef VIR_NODISCARD
#endif  // VIR_LOCKFREE_RING_H_
