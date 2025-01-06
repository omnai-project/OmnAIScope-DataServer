# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/anna/miniBE/build/_deps/cmake_git_version-src"
  "/home/anna/miniBE/build/_deps/cmake_git_version-build"
  "/home/anna/miniBE/build/_deps/cmake_git_version-subbuild/cmake_git_version-populate-prefix"
  "/home/anna/miniBE/build/_deps/cmake_git_version-subbuild/cmake_git_version-populate-prefix/tmp"
  "/home/anna/miniBE/build/_deps/cmake_git_version-subbuild/cmake_git_version-populate-prefix/src/cmake_git_version-populate-stamp"
  "/home/anna/miniBE/build/_deps/cmake_git_version-subbuild/cmake_git_version-populate-prefix/src"
  "/home/anna/miniBE/build/_deps/cmake_git_version-subbuild/cmake_git_version-populate-prefix/src/cmake_git_version-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/anna/miniBE/build/_deps/cmake_git_version-subbuild/cmake_git_version-populate-prefix/src/cmake_git_version-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/anna/miniBE/build/_deps/cmake_git_version-subbuild/cmake_git_version-populate-prefix/src/cmake_git_version-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
