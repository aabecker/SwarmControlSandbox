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