# Argus-camera
using libargus to get camera metadata

for r28 version in ubuntu16.04

modify in /utils /include and src/Dispathcher.cpp(h) src/CameraAPI.cpp(h)


problem: 
   in src/Dispathcher.cpp function void Dispatcher::framedataMap(int devIndex)
   u、v mmap will be failed 
   nvbuffer format for YUV is unkown ,to be done ...
