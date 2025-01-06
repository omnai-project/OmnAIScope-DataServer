# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/anna/miniBE/vcpkg"
  "/home/anna/miniBE/build/vcpkg-build"
  "/home/anna/miniBE/build/vcpkg-subbuild/vcpkg-populate-prefix"
  "/home/anna/miniBE/build/vcpkg-subbuild/vcpkg-populate-prefix/tmp"
  "/home/anna/miniBE/build/vcpkg-subbuild/vcpkg-populate-prefix/src/vcpkg-populate-stamp"
  "/home/anna/miniBE/build/vcpkg-subbuild/vcpkg-populate-prefix/src"
  "/home/anna/miniBE/build/vcpkg-subbuild/vcpkg-populate-prefix/src/vcpkg-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/anna/miniBE/build/vcpkg-subbuild/vcpkg-populate-prefix/src/vcpkg-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/anna/miniBE/build/vcpkg-subbuild/vcpkg-populate-prefix/src/vcpkg-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
