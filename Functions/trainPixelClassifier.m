function [trainedClassifier, validationAccuracy] = trainPixelClassifier(trainingData)
% [trainedClassifier, validationAccuracy] = trainPixelClassifier(trainingData)
% 学習済み分類器とその 精度 を返します。このコードは分類学習器アプリで学習させた分類モデル
% を再作成します。生成されるコードを使用して、同じモデルでの新規データを使用した学習の自動化
% や、プログラミングによってモデルに学習させる方法の調査を行います。
%
%  入力:
%      trainingData: [R,G,B,Label]
%
%  出力:
%      trainedClassifier: 学習済みの分類器を含む struct。この struct には、学習済み
%       の分類器に関する情報をもつさまざまなフィールドが含まれています。
%
%      trainedClassifier.predictFcn: 新規データに関する予測を行う関数。
%
%      validationAccuracy: パーセント単位の精度を表す double。アプリでは [履歴] リ
%       ストにこの全体的な精度スコアがモデルごとに表示されます。
%
% このコードを使用して新規データでモデルに学習させます。分類器に再学習させるには、元のデータ
% または新規データを入力引数 trainingData として指定して、コマンド ラインから関数を呼び
% 出します。
%
% たとえば、元のデータセット T で学習させた分類器に再学習させるには、次を入力します:
%   [trainedClassifier, validationAccuracy] = trainClassifier(T)
%
% 返された 'trainedClassifier' を使用して新規データ T2 の予測を行うには、次を使用しま
% す
%   yfit = trainedClassifier.predictFcn(T2)
%
% T2 は、少なくとも学習中に使用したものと同じ予測列を含むテーブルでなければなりません。詳細
% については、次のように入力してください:
%   trainedClassifier.HowToPredict

% MATLAB からの自動生成日: 2021/06/04 15:49:17


% 予測子と応答の抽出
% このコードは、データを処理して、モデルに学習させるのに適した
% 形状にします。
inputTable = trainingData;
predictorNames = {'Var1', 'Var2', 'Var3'};
predictors = inputTable(:, predictorNames);
response = inputTable.Var4;
isCategoricalPredictor = [false, false, false];

% 分類器の学習
% このコードは、すべての分類器オプションを指定し、分類器に学習させます。
% ロジスティック回帰では、応答値は二項分布に従うと仮定されます。
successClass = true;
failureClass = false;
% 多数応答のクラスを計算します。fitglm からの NaN 予測がある場合は、NaN をこの多数クラスのラベルに変換します。
numSuccess = sum(response);
numFailure = sum(~response);
if numSuccess > numFailure
    missingClass = successClass;
else
    missingClass = failureClass;
end
successFailureAndMissingClasses = [successClass; failureClass; missingClass];
zeroOneResponse = response;
% fitglm の入力引数を準備します。
concatenatedPredictorsAndResponse = [predictors, table(zeroOneResponse)];
% fitglm を使用して学習させます。
GeneralizedLinearModel = fitglm(...
    concatenatedPredictorsAndResponse, ...
    'Distribution', 'binomial', ...
    'link', 'logit');

% 予測確率を、予測されるクラスのラベルとスコアに変換します。
convertSuccessProbsToPredictions = @(p) successFailureAndMissingClasses( ~isnan(p).*( (p<0.5) + 1 ) + isnan(p)*3 );
returnMultipleValuesFcn = @(varargin) varargin{1:max(1,nargout)};
scoresFcn = @(p) [1-p, p];
predictionsAndScoresFcn = @(p) returnMultipleValuesFcn( convertSuccessProbsToPredictions(p), scoresFcn(p) );

% 関数 predict で結果の構造体を作成
predictorExtractionFcn = @(t) t(:, predictorNames);
logisticRegressionPredictFcn = @(x) predictionsAndScoresFcn( predict(GeneralizedLinearModel, x) );
trainedClassifier.predictFcn = @(x) logisticRegressionPredictFcn(predictorExtractionFcn(x));

% 結果の構造体にさらにフィールドを追加
trainedClassifier.RequiredVariables = {'Var1', 'Var2', 'Var3'};
trainedClassifier.GeneralizedLinearModel = GeneralizedLinearModel;
trainedClassifier.SuccessClass = successClass;
trainedClassifier.FailureClass = failureClass;
trainedClassifier.MissingClass = missingClass;
trainedClassifier.ClassNames = {successClass; failureClass};
trainedClassifier.About = 'この構造体は、分類学習器 R2020a からエクスポートされた学習済みのモデルです。';
trainedClassifier.HowToPredict = sprintf('新しいテーブル T についての予測を行うには、次を使用します: \n yfit = c.predictFcn(T) \n''c'' をこの構造体の変数の名前 (''trainedModel'' など) に置き換えます。 \n \nテーブル T は次によって返される変数を含んでいなければなりません: \n c.RequiredVariables \n変数形式 (行列/ベクトル、データ型など) は元の学習データと一致しなければなりません。 \n追加の変数は無視されます。 \n \n詳細については、<a href="matlab:helpview(fullfile(docroot, ''stats'', ''stats.map''), ''appclassification_exportmodeltoworkspace'')">How to predict using an exported model</a> を参照してください。');

% 予測子と応答の抽出
% このコードは、データを処理して、モデルに学習させるのに適した
% 形状にします。
inputTable = trainingData;
predictorNames = {'Var1', 'Var2', 'Var3'};
predictors = inputTable(:, predictorNames);
response = inputTable.Var4;
isCategoricalPredictor = [false, false, false];

% 交差検証の実行
KFolds = 5;
cvp = cvpartition(response, 'KFold', KFolds);
% 予測を適切なサイズに初期化
validationPredictions = response;
numObservations = size(predictors, 1);
numClasses = 2;
validationScores = NaN(numObservations, numClasses);
for fold = 1:KFolds
    trainingPredictors = predictors(cvp.training(fold), :);
    trainingResponse = response(cvp.training(fold), :);
    foldIsCategoricalPredictor = isCategoricalPredictor;
    
    % 分類器の学習
    % このコードは、すべての分類器オプションを指定し、分類器に学習させます。
    % ロジスティック回帰では、応答値は二項分布に従うと仮定されます。
    successClass = true;
    failureClass = false;
    % 多数応答のクラスを計算します。fitglm からの NaN 予測がある場合は、NaN をこの多数クラスのラベルに変換します。
    numSuccess = sum(trainingResponse);
    numFailure = sum(~trainingResponse);
    if numSuccess > numFailure
        missingClass = successClass;
    else
        missingClass = failureClass;
    end
    successFailureAndMissingClasses = [successClass; failureClass; missingClass];
    zeroOneResponse = trainingResponse;
    % fitglm の入力引数を準備します。
    concatenatedPredictorsAndResponse = [trainingPredictors, table(zeroOneResponse)];
    % fitglm を使用して学習させます。
    GeneralizedLinearModel = fitglm(...
        concatenatedPredictorsAndResponse, ...
        'Distribution', 'binomial', ...
        'link', 'logit');
    
    % 予測確率を、予測されるクラスのラベルとスコアに変換します。
    convertSuccessProbsToPredictions = @(p) successFailureAndMissingClasses( ~isnan(p).*( (p<0.5) + 1 ) + isnan(p)*3 );
    returnMultipleValuesFcn = @(varargin) varargin{1:max(1,nargout)};
    scoresFcn = @(p) [1-p, p];
    predictionsAndScoresFcn = @(p) returnMultipleValuesFcn( convertSuccessProbsToPredictions(p), scoresFcn(p) );
    
    % 関数 predict で結果の構造体を作成
    logisticRegressionPredictFcn = @(x) predictionsAndScoresFcn( predict(GeneralizedLinearModel, x) );
    validationPredictFcn = @(x) logisticRegressionPredictFcn(x);
    
    % 結果の構造体にさらにフィールドを追加
    
    % 検証予測の計算
    validationPredictors = predictors(cvp.test(fold), :);
    [foldPredictions, foldScores] = validationPredictFcn(validationPredictors);
    
    % 予測を元の順序で保存
    validationPredictions(cvp.test(fold), :) = foldPredictions;
    validationScores(cvp.test(fold), :) = foldScores;
end

% 検証精度の計算
correctPredictions = (validationPredictions == response);
validationAccuracy = sum(correctPredictions)/length(correctPredictions);
