function [fitresult, gof] = createFit(YdegHisU, YdegHisW, Yerror)
%CREATEFIT(YDEGHISU,YDEGHISW,YERROR)
%  近似を作成します。
%
%  'Yestimate' に対するデータを近似:
%      X 入力:  YdegHisU
%      Y 入力:  YdegHisW
%      Z 出力:  Yerror
%  出力:
%      fitresult: 近似を表す fit オブジェクト。
%      gof: 適合性情報をもつ構造体。
%
%  参考 FIT, CFIT, SFIT.

%  MATLAB からの自動生成日: 03-Feb-2022 12:40:23


%% 近似: 'Yestimate'。
[xData, yData, zData] = prepareSurfaceData( YdegHisU, YdegHisW, Yerror );

% 近似タイプとオプションを設定します。
ft = 'thinplateinterp';

% モデルをデータに近似します。
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% データの近似をプロットします。
figure( 'Name', 'Yestimate' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'Yestimate', 'Yerror vs. YdegHisU, YdegHisW', 'Location', 'NorthEast', 'Interpreter', 'none' );
% ラベル Axes
xlabel( 'YdegHisU', 'Interpreter', 'none' );
ylabel( 'YdegHisW', 'Interpreter', 'none' );
zlabel( 'Yerror', 'Interpreter', 'none' );
grid on
view( -50.0, -2.0 );


