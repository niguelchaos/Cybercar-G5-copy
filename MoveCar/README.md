## To Deploy MoveCar:

* Step 1: Be on the MoveCar/ directory. (Not in src)

* Step 2: to build for car,
```bash
docker build -t movecar/<whatever-name>.armhf -f Dockerfile.armhf .
```

* Step 3:
```bash
docker save movecar/<whatever-name>.armhf > movecar-<whatever-name>.armhf.tar
```

* Step 4: Copy tar file to car
```
scp -P 2200 movecar-<whatever-name>.armhf.tar
```

* Step 5: Load on car
```
cat movecar-<whatever-name>.armhf.tar | docker load
```

* Step 6: RUN
```
docker run --rm --init --net=host movecar/<whatever-name>.armhf 
```
