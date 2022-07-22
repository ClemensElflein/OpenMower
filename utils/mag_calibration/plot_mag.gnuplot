#f(x,y) = (x-bx)*(x-bx) + (y-by)*(y-by) - r
#fit f(x, y) '/tmp/mag.csv' u 5:6:(0) via bx,by,r

#set xr [-50:50]
#set yr [-50:50]

#set contour
#set view map
#unset surface
#set cntrparam levels discrete 0
#set isosamples 1000,1000

#set table '/tmp/contour.dat'
#splot f(x, y)

#unset table
#unset contour

#stats '/tmp/mag.csv' using 9:10

set grid
set size square


plot \
'/tmp/mag.csv' using 5:6 with points
