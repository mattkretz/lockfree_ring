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

#include "unittest.h"
#include "../lockfree_ring.h"
#include "metahelpers.h"

struct Data {
  static std::atomic<size_t> ctor, dtor, copied, moved;
  static size_t alive() { return ctor + copied + moved - dtor; }
  size_t x;
  Data(size_t y) noexcept : x(y) { ++ctor; }
  Data(const Data &rhs) noexcept : x(rhs.x) { ++copied; }
  Data(Data &&rhs) noexcept : x(rhs.x) { ++moved; }
  ~Data() { ++dtor; }
};
std::atomic<size_t> Data::ctor{0};
std::atomic<size_t> Data::dtor{0};
std::atomic<size_t> Data::copied{0};
std::atomic<size_t> Data::moved{0};

template <size_t N> using _ = std::integral_constant<size_t, N>;

TEST_BEGIN(NN, foo,
           (_<1>, _<2>, _<4>, _<8>, _<0x10>, _<0x20>, _<0x40>, _<0x80>, _<0x100>,
            _<0x200>, _<0x400>, _<0x800>, _<0x1000>, _<0x2000>, _<0x4000>, _<0x8000>))
{
  Data::ctor = 0;
  Data::dtor = 0;
  Data::copied = 0;
  Data::moved = 0;
  vir::lockfree_ring<Data, NN::value> ring;
  for (size_t i = 0; i <= NN::value; ++i) {
    VERIFY(ring.prepare_push(Data{i + 1}).try_push());
    auto popped = ring.pop_front().get();
    VERIFY(bool(popped));
    COMPARE(popped->x, i + 1);
  }

  for (size_t i = 0; i < NN::value; ++i) {
    VERIFY(ring.prepare_push(Data{i + 1}).try_push());
  }

  {
    auto pusher = ring.prepare_push(Data{NN::value + 1});
    VERIFY(!pusher.try_push());
    {
      auto popped = ring.pop_front().get();
      VERIFY(bool(popped));
      COMPARE(popped->x, 1u);
      VERIFY(pusher.try_push());
      COMPARE(size_t(Data::alive()), NN::value + 1u);
    }
    COMPARE(size_t(Data::alive()), NN::value);
  }
}
COMPARE(size_t(Data::alive()), 0u);
COMPARE(size_t(Data::ctor), 2u * NN::value + 2);
COMPARE(size_t(Data::copied), 0u);
TEST_END

template <class T> struct Caller {
  template <class U>
  decltype(std::declval<T &>().prepare_push(std::declval<U>())) operator()(U &&);
};

TEST(traits)
{
  VERIFY( std::is_copy_constructible<Data>::value);
  VERIFY( std::is_move_constructible<Data>::value);
  VERIFY( std::is_nothrow_move_constructible<Data>::value);

  {
    using T = vir::lockfree_ring<Data, 2>;
    VERIFY(!std::is_copy_constructible<T>::value);
    VERIFY(std::is_move_constructible<T>::value);
    VERIFY(std::is_nothrow_move_constructible<T>::value);
    VERIFY(std::is_nothrow_default_constructible<T>::value);
    VERIFY(noexcept(std::declval<T &>().pop_front()));
    VERIFY(noexcept(std::declval<T &>().pop_front().get()));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<Data>())));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<Data>()).try_push()));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<const Data &>())));
    VERIFY(noexcept(
        std::declval<T &>().prepare_push(std::declval<const Data &>()).try_push()));
  }
  {
    struct A {
      A(const A &rhs) noexcept(false);
      A(A &&rhs) noexcept(false);
    };
    using T = vir::lockfree_ring<A, 2>;
    VERIFY(!std::is_copy_constructible<T>::value);
    VERIFY(std::is_move_constructible<T>::value);
    VERIFY(!std::is_nothrow_move_constructible<T>::value);
    VERIFY(std::is_nothrow_default_constructible<T>::value);
    VERIFY(noexcept(std::declval<T &>().pop_front()));
    VERIFY(!noexcept(std::declval<T &>().pop_front().get()));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<A>())));
    VERIFY(!noexcept(std::declval<T &>().prepare_push(std::declval<A>()).try_push()));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<const A &>())));
    VERIFY(!noexcept(
        std::declval<T &>().prepare_push(std::declval<const A &>()).try_push()));
  }
  {
    struct A {
      A(const A &rhs) = delete;
      A(A &&rhs) noexcept(false);
    };
    using T = vir::lockfree_ring<A, 2>;
    VERIFY(!std::is_copy_constructible<T>::value);
    VERIFY(std::is_move_constructible<T>::value);
    VERIFY(!std::is_nothrow_move_constructible<T>::value);
    VERIFY(std::is_nothrow_default_constructible<T>::value);
    VERIFY(noexcept(std::declval<T &>().pop_front()));
    VERIFY(!noexcept(std::declval<T &>().pop_front().get()));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<A>())));
    VERIFY(!noexcept(std::declval<T &>().prepare_push(std::declval<A>()).try_push()));
    VERIFY(!sfinae_is_callable<const A &>(Caller<T>()));
  }
  {
    struct A {
      A(const A &rhs) noexcept;
    };
    using T = vir::lockfree_ring<A, 2>;
    VERIFY(!std::is_copy_constructible<T>::value);
    VERIFY(std::is_move_constructible<T>::value);
    VERIFY(std::is_nothrow_move_constructible<T>::value);
    VERIFY(std::is_nothrow_default_constructible<T>::value);
    VERIFY(noexcept(std::declval<T &>().pop_front()));
    VERIFY(noexcept(std::declval<T &>().pop_front().get()));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<A>())));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<A>()).try_push()));
    VERIFY(noexcept(std::declval<T &>().prepare_push(std::declval<const A &>())));
    VERIFY(
        noexcept(std::declval<T &>().prepare_push(std::declval<const A &>()).try_push()));
  }
}
