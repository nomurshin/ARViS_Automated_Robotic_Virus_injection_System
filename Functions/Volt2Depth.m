function depth = Volt2Depth(volt,linearmodel)
%VOLT2DEPTH レーザーセンサからの電圧データを表示されている数値に変換する。
%   電圧と表示の数値には誤差が存在し、その誤差は電圧の大きさに比例する。この関数
%   でその誤差を修正する。
p1 = linearmodel.p1;
p2 = linearmodel.p2;
volt = (volt-2.5)*2;
depth = volt + p1*volt + p2;
end

