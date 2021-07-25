# albireo-pi

## build
```
pipenv sync
pipenv run cmake -G Ninja -B build -D CMAKE_BUILD_TYPE=Debug
ninja -v -C build
```
## run
```
sudo ./build/bin/main
```
