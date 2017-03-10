project(lockfree_ring)
cmake_minimum_required(VERSION 2.6)
enable_testing()
add_subdirectory(tests)

install(FILES lockfree_ring.h DESTINATION include/vir/)
