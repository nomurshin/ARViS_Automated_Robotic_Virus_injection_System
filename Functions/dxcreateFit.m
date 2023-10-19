function [fitresult, gof] = dxcreateFit(b, a)
%CREATEFIT(B,A)
%  近似を作成します。
%
%  '新規近似 1' に対するデータを近似:
%      X 入力:  b
%      Y 出力:  a
%  出力:
%      fitresult: 近似を表す fit オブジェクト。
%      gof: 適合性情報をもつ構造体。
%
%  参考 FIT, CFIT, SFIT.

%  MATLAB からの自動生成日: 14-Feb-2022 16:40:19


%% 近似: '新規近似 1'。
[xData, yData] = prepareCurveData( b, a );

% 近似タイプとオプションを設定します。
ft = fittype( 'poly1' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Robust = 'LAR';

% モデルをデータに近似します。
[fitresult, gof] = fit( xData, yData, ft, opts );

% データの近似をプロットします。
figure( 'Name', '新規近似 1' );
h = plot( fitresult, xData, yData );
legend( h, 'a vs. b', '新規近似 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% ラベル Axes
xlabel( 'b', 'Interpreter', 'none' );
ylabel( 'a', 'Interpreter', 'none' );
grid on


