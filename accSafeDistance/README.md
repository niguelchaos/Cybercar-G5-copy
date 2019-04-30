## To Deploy accSafeDistance:

* Step 1: Be on the accSafeDistance/ directory. (Not in src)

* Step 2: to build for car,
```bash
docker build -t safedist/<whatever-name>.armhf -f Dockerfile.armhf .
```

* Step 3:
```bash
docker save safedist/<whatever-name>.armhf > safedist-<whatever-name>.armhf.tar
```

* Step 4: Copy tar file to car
```
scp -P 2200 safedist-<whatever-name>.armhf.tar
```

* Step 5: Load on car
```
cat safedist-<whatever-name>.armhf.tar | docker load
```

* Step 6: RUN
```
docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp safedist/<whatever-name>.armhf --cid=112 --name=img.argb --width=640 --height=480
```


### Local testing
1. In localtest/ folder, make a build folder.

2. in the build folder, type ```cmake ..```
3. then, ```make```
4. then, ```./local-safe-distance```
