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