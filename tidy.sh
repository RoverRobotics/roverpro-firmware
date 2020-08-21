#!/bin/sh

clang-format -i --style=file $(git ls-files "PowerBoard/**/*.[ch]" "bootypic/**/*.[ch]")
