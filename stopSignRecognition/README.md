To Deploy stopsign recognition:

Step 1: Be on the stopSignRecognition/ directory. (Not in src), where the docker files are and open the terminal there.

Step 2: build the image for the car:
docker build -t whatevername.armhf -f Dockerfile.armhf .

Step 3: safe the docker image:
docker save whatevername.armhf > whatevername.armhf.tar

Step 4: Copy the tar file into the car's raspberry pi using secure copy:
scp -P 2200 whatevername.armhf.tar pi@192.168.8.1:~

Step 5: ssh into the raspberry pi:
ssh -p 2200 pi@192.168.8.1

Step 6: Load the image on the car:
cat whatevername.armhf.tar | docker load

Step 6: RUN the image:
docker run --rm -ti --init --net=host --ipc=host -v /tmp:/tmp whatevername.armhf --cid=112 --name=img.argb --width=640 --height=480

/////////////////////////////////////////////////////////////////////////////
For Local testing with the computer camera:

Step 1: In /stopSignRecognition/src/localtest folder, make a build folder.

Step 2: in the build folder, type:
cmake .. (do not forget the two dots at the end):

Step 3: type:
make

Step 4: ./local-stop-sign
/////////////////////////////////////////////////////////
