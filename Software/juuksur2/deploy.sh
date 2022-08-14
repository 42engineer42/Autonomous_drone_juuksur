git add -u
git commit -m "automated commit"
git push pi@192.168.4.1:/home/pi/juuksur2
ssh pi@192.168.4.1 "cd ~/juuksur2 && source devel/setup.bash && catkin_make -j1 -DCMAKE_BUILD_TYPE=Release"
