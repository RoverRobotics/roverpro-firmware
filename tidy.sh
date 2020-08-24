#!/bin/sh

clang-format -i --style=file $(git ls-files "PowerBoard/**/*.[ch]" "bootypic/**/*.[ch]")
clang-tidy --fix-errors -p=build --format-style=file $(git ls-files "PowerBoard/**/*.[ch]")
