function Gradient(x,y)
{  // computes the x and y gradient of cell(x,y)  requires a map with distance values.
	var dist = map[x][y];
	if(dist == -1)
	{
		gradient_x[x][y] = 0;
		gradient_y[x][y] = 0;
		return;
	}
	
	if(map[x-1][y] == -1 && map[x+1][y] == -1)
	{
		gradient_x[x][y] = 0;
	}
	else if(map[x-1][y] == -1 )
	{
		gradient_x[x][y]= map[x+1][y] - map[x][y];
	}
	else if(map[x+1][y] == -1 )
	{
		gradient_x[x][y]= map[x][y] - map[x-1][y];
	}
	else
	{
		gradient_x[x][y] = (map[x+1][y] - map[x-1][y])/2;
	}
	if(map[x][y-1] == -1 && map[x][y+1] == -1)
	{
		gradient_y[x][y] = 0;
	}
	else if(map[x][y-1] == -1 )
	{
		gradient_y[x][y]= 0;//map[x][y+1] - map[x][y];
	}
	else if(map[x][y+1] == -1 )
	{
		gradient_y[x][y]= map[x][y] - map[x][y-1];
	}
	else
	{
		gradient_y[x][y] = (map[x][y+1] - map[x][y-1])/2;
	}
	if(gradient_x[x][y] != 0 && gradient_y[x][y] != 0)
	{
		gradient_angle[x][y] = Math.atan2(gradient_y[x][y],gradient_x[x][y] ) * 180 / Math.PI + 270;
	}
}