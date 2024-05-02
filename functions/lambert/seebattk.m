% ------- recursion algorithms needed by the lambertbattin routine
function k = seebattk(v)
%-----------------------------  Locals  ------------------------------

%--------------------  Implementation   ----------------------
d(1) =     1.0D0 /    3.0D0;
d(2) =     4.0D0 /   27.0D0;
d(3) =     8.0D0 /   27.0D0;
d(4) =     2.0D0 /    9.0D0;
d(5) =    22.0D0 /   81.0D0;
d(6) =   208.0D0 /  891.0D0;
d(7) =   340.0D0 / 1287.0D0;
d(8) =   418.0D0 / 1755.0D0;
d(9) =   598.0D0 / 2295.0D0;
d(10) =   700.0D0 / 2907.0D0;
d(11)=   928.0D0 / 3591.0D0;
d(12)=  1054.0D0 / 4347.0D0;
d(13)=  1330.0D0 / 5175.0D0;
d(14)=  1480.0D0 / 6075.0D0;
d(15)=  1804.0D0 / 7047.0D0;
d(16)=  1978.0D0 / 8091.0D0;
d(17)=  2350.0D0 / 9207.0D0;
d(18)=  2548.0D0 /10395.0D0;
d(19)=  2968.0D0 /11655.0D0;
d(20)=  3190.0D0 /12987.0D0;
d(21)=  3658.0D0 /14391.0D0;

% ----------------- Process Forwards ------------------------
sum1   = d(1);
delold = 1.0D0; 
termold= d(1);
i      = 1;
while(1)
    del  = 1.0D0 / ( 1.0D0 + d(i+1)*v*delold );
    term = termold * ( del - 1.0D0 );
    sum1 = sum1 + term;
    i    = i + 1;
    delold = del;
    termold= term;
    if ((i<=20) || (abs(Termold) > 0.000001D0 ))
        break
    end
end

k= sum1;

