function BFS(x, y ) 
{  //recursively compute Breadth First Search from goal to every other grid cell
	var dist = map[x][y];
	if(dist == -1 ) // this is an obstacle region
		{return;}
	if(x-1 >0)
	{
		if(map[x-1][y] > dist +1)
		{
			map[x-1][y] = dist+1;
			//console.log("with " + x-1 + " and " + y + " We have : " + map[x-1][y]);
			BFS(x-1, y);
		}
	}
	if(y+1 < mapYSize)
	{
		if(map[x][y+1] > dist +1)
		{
			map[x][y+1] = dist+1;
			//console.log("with " + x + " and " + y +1+ " We have : " + map[x][y+1]);
			BFS(x, y+1);
		}
	}
	if(y-1 >0)
	{
		if(map[x][y-1] > dist +1)
		{
			map[x][y-1] = dist+1;
			//console.log("with " + x + " and " + y-1 + " We have : " + map[x][y-1]);
			BFS(x, y-1);
		}
	}
	if(x+1 < mapXSize)
	{
		if(map[x+1][y] > dist +1)
		{
			map[x+1][y] = dist+1;
			//console.log("with " + x+1 + " and " + y + " We have : " + map[x+1][y]);
			BFS(x+1, y);
		}
	}
}


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


function calcPointsCirc( cx,cy, rad, dashLength)
{	//draw a dashed circle
	var n = rad/dashLength,
	alpha = Math.PI * 2 / n,
	pointObj = {},
	points = [],
	i = -1;
	while( i < n )
	{
		var theta = alpha * i,
		theta2 = alpha * (i+1);
		points.push({x : (Math.cos(theta) * rad) + cx, y : (Math.sin(theta) * rad) + cy, ex : (Math.cos(theta2) * rad) + cx, ey : (Math.sin(theta2) * rad) + cy});
		i+=2;
	}              
	return points;            
}

function lineDistance( x1,y1,x2,y2)
	{
		var xs = 0;
		var ys = 0;
		xs = x1 - x2;
		xs = xs * xs;
		ys = y1 - y2;
		ys = ys * ys;
		return Math.sqrt( xs + ys );
	}


