for i=1:size(drawTime,1)
    if drawTime(i,1) >0
        drawTime(i,1) = 90- drawTime(i,1)*180/pi;
    else
        drawTime(i,1) = -90- drawTime(i,1)*180/pi;
    end

end