echo
echo ---Building batotp library---
mkdir -p Release
cd batotp
echo compiling spline.cpp...
g++ -Wall -Wextra -std=c++11 -O3 -s -DNDEBUG -c spline.cpp -o spline.o
echo compiling util.cpp...
g++ -Wall -Wextra -std=c++11 -O3 -s -DNDEBUG -c util.cpp -I../../eigenlib -I/usr/include/eigen3 -o util.o
echo compiling robot.cpp...
g++ -Wall -Wextra -std=c++11 -O3 -s -DNDEBUG -c robot.cpp -I../../eigenlib -I/usr/include/eigen3 -o robot.o
echo compiling ba.cpp...
g++ -Wall -Wextra -std=c++11 -O3 -s -DNDEBUG -c ba.cpp -o ba.o
echo making libbatotp.a...
ar rvs ../Release/libbatotp.a spline.o util.o robot.o ba.o

echo
echo ---Building batest executable---
cd ../test
echo compiling main.cpp...
g++ -Wall -Wextra -std=c++11 -O3 -s -DNDEBUG -c main.cpp -I../batotp -lbatotp -o main.o
echo making batest...
g++ -Wall -Wextra -std=c++11 -O3 -s -DNDEBUG main.o -L../Release -lbatotp -o ../Release/batest
echo
echo Finished. The executable is Release/batest
echo

cd ..
