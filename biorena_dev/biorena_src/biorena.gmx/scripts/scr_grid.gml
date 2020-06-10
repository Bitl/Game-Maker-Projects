draw_set_color(gridcolor)
draw_set_alpha(gridalpha)
hor_sep=argument0
vert_sep=argument1
x1=argument2
y1=argument3
x2=argument4
y2=argument5

var i,ii;
i=0
ii=0

repeat((y1+y2)/hor_sep)
{
  draw_line(x1,hor_sep*i,x2,hor_sep*i)
  i+=1
}

repeat((x1+x2)/vert_sep)
{
  draw_line(vert_sep*ii,y1,vert_sep*ii,y2)
  ii+=1
}

draw_set_alpha(1)
