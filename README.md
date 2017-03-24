Lock-free queue using a ring buffer on the stack
==================================================

[![license](https://img.shields.io/github/license/mattkretz/lockfree_ring.svg)](https://github.com/mattkretz/lockfree_ring/blob/master/LICENSE)
[![language](https://img.shields.io/badge/language-C%2B%2B11-blue.svg)](https://isocpp.org/)
[![depends](https://img.shields.io/badge/depends-virtest-brightgreen.svg)](https://github.com/mattkretz/virtest)

[![Build Status](https://travis-ci.org/mattkretz/lockfree_ring.svg)](https://travis-ci.org/mattkretz/lockfree_ring)
[![Build status](https://ci.appveyor.com/api/projects/status/5k3wkpxvoqf99ef6?svg=true)](https://ci.appveyor.com/project/mattkretz/lockfree_ring)

Example
--------------

```C++
#include <lockfree_ring.h>

// consumer threads:
void consumer(vir::lockfree_ring<Data> &q)
{
  while (keep_running) {
    auto popper = q.pop_front();
    for (;;) {
      std::optional<Data> data = popper.get();
      if (data) {
        do_work(data.value);
        break;
      }
      do_something_else();
    }
  }
}

// producer threads:
void producer(vir::lockfree_ring<Data> &q)
{
  while (keep_running) {
    Data data = produce_data();
    auto pusher = q.prepare_push(std::move(data));
    while (!pusher.try_push()) {
       do_something_else();
    }
  }
}
```

Remarks
--------------

There is little guarantee about ordering and no guarantee about fairness.  
However, if the size of the ring is larger than the number of active reader 
objects (returned from `pop_front()`) then all consumers will eventually be 
able to read a value (unless one of the writers does not fill its `pusher` 
object).
