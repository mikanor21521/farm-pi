# Albireo farm pi

## Clone

```
git clone https://github.com/41b1r30/farm-pi.git
cd farm-pi
git checkout experiment/water_temperature
git submodule update --init
```

## Build

```
cmake .
make
sudo ./main
```

## Format

```
clang-format --style=file -i ./src/**.cpp ./include/**.hpp
cmake-format -i ./CMakeLists.txt
```

## Lint

```
cmake . -D CMAKE_EXPORT_COMPILE_COMMANDS=ON
clang-tidy ./src/**.cpp
cmake-lint ./CMakeLists.txt
```
