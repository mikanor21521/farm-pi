# Albireo farm pi

## Build

```
cmake -G Ninja -B build -D BUILD_SHARED_LIBS=ON -D CMAKE_BUILD_TYPE=Debug
ninja -v -C build
sudo ./build/main
```

