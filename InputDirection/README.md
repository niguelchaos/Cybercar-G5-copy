## To Deploy InputDirection:

* Step 1: Be on the InputDirection/ directory. (Not in src)

* Step 2: to build for car,
```bash
docker build -f Dockerfile.armhf -t <some-name> .
```

* Step 3:
```bash
docker save movecar/<some-name> > <some-name>.tar
```

* Step 4: Copy tar file to car
```
scp -P 2200 <some-name>.tar  pi@192.168.8.1:~/
```

* Step 5: Load on car
```
docker load < some-name.tar 
<or this>
cat movecar-<whatever-name>.armhf.tar | docker load
```

* Step 6: RUN
```
docker run --rm -ti --net=host movecar/<some-name> 
```
