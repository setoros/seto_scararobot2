var Stage; //描画領域

var StageSize = 500; //描画エリアサイズ
var BeadsStageScale = 29; //一列のビーズ数
var BeadsCount = BeadsStageScale * BeadsStageScale; //ビーズの総数
var BeadsSize = Math.floor(StageSize / BeadsStageScale); //ビーズの描画サイズ

var is8x8 = true; //8x8モードの場合はtrue、falseで29x29モード

var BackgroundColor = '#DDDDDD'; //背景色
var GridColor = '#BBBBBB'; //グリッドの色

var Offset_X = 21;
var Offset_Y = 0;

var DrawAreaX = BeadsSize * Offset_X; //描画可能領域 X座標
var DrawAreaY = BeadsSize * Offset_Y;; //描画可能領域 Y座標
var DrawAreaW = BeadsSize * 8; //描画可能領域 幅
var DrawAreaH = BeadsSize * 8; //描画可能領域 高さ
var DrawAreaStrokeWidth = 2; //描画可能領域 線幅

var ToolList ={}; //ツールリスト
ToolList.pen = 1;
ToolList.eraser = 2;

var CurrentTool = ToolList.pen; //選択中のツール

var BeadsSeq = 0;

var BeadsBitmapData =  new createjs.BitmapData(null, StageSize, StageSize); //ビーズ設置ビットマップデータ
var BeadsBitmap = new createjs.Bitmap(BeadsBitmapData.canvas); //ビットマップオブジェクト

//ビーズ配置データの1次元配列
var BeadsData = [];
initBeadsData();

var isBeadsDataDefined = false; //ページロード時にビーズ配置済みフラグ

//カラーパレット
var ColorList = [];
//赤
ColorList.push({'Id': '1', 'Color': '#d10011', 'isTransparent': false});
ColorList.push({'Id': '2', 'Color': '#76c789', 'isTransparent': false});
ColorList.push({'Id': '3', 'Color': '#623882', 'isTransparent': false});
ColorList.push({'Id': '4', 'Color': '#df852d', 'isTransparent': false});
ColorList.push({'Id': '5', 'Color': '#f67700', 'isTransparent': false});

ColorList.push({'Id': '6', 'Color': '#8b8a8e', 'isTransparent': false});
ColorList.push({'Id': '7', 'Color': '#1791c8', 'isTransparent': false});
ColorList.push({'Id': '8', 'Color': '#e52f81', 'isTransparent': false});
ColorList.push({'Id': '9', 'Color': '#eac005', 'isTransparent': false});
ColorList.push({'Id': 'A', 'Color': '#e73e95', 'isTransparent': false});

ColorList.push({'Id': 'B', 'Color': '#ece9e4', 'isTransparent': false});
ColorList.push({'Id': 'C', 'Color': '#090808', 'isTransparent': false});
ColorList.push({'Id': 'D', 'Color': '#8c481c', 'isTransparent': false});
ColorList.push({'Id': 'E', 'Color': '#4e2217', 'isTransparent': false});
ColorList.push({'Id': 'F', 'Color': '#007431', 'isTransparent': false});

ColorList.push({'Id': 'G', 'Color': '#af53a5', 'isTransparent': false});
ColorList.push({'Id': 'H', 'Color': '#6cbd03', 'isTransparent': false});
ColorList.push({'Id': 'I', 'Color': '#00a294', 'isTransparent': false});
ColorList.push({'Id': 'J', 'Color': '#00bf4a', 'isTransparent': false});
ColorList.push({'Id': 'K', 'Color': '#fa2933', 'isTransparent': false});

ColorList.push({'Id': 'L', 'Color': '#9dd7d8', 'isTransparent': false});
ColorList.push({'Id': 'M', 'Color': '#024390', 'isTransparent': false});
ColorList.push({'Id': 'N', 'Color': '#cecb00', 'isTransparent': false});
ColorList.push({'Id': 'O', 'Color': '#fc7a68', 'isTransparent': false});
ColorList.push({'Id': 'P', 'Color': '#9ba6a6', 'isTransparent': true});

var IndexList = {};

IndexList['1'] = 0;
IndexList['2'] = 1;
IndexList['3'] = 2;
IndexList['4'] = 3;
IndexList['5'] = 4;

IndexList['6'] = 5;
IndexList['7'] = 6;
IndexList['8'] = 7;
IndexList['9'] = 8;
IndexList['A'] = 9;

IndexList['B'] = 10;
IndexList['C'] = 11;
IndexList['D'] = 12;
IndexList['E'] = 13;
IndexList['F'] = 14;

IndexList['G'] = 15;
IndexList['H'] = 16;
IndexList['I'] = 17;
IndexList['J'] = 18;
IndexList['K'] = 19;

IndexList['L'] = 20;
IndexList['M'] = 21;
IndexList['N'] = 22;
IndexList['O'] = 23;
IndexList['P'] = 24;

var CurrentColor = null; //現在選択中の色
var CurrentColorId = null; //現在選択中の色ID

/**
 * 初期化
 * @returns {undefined}
 */
$(function(){

    Stage = new createjs.Stage('beads_canvas');

    var bg = new createjs.Shape();
    bg.graphics.beginFill(BackgroundColor).drawRect(0, 0, StageSize, StageSize);
    bg.x = 0;
    bg.y = 0;
    Stage.addChild(bg);
    Stage.addChild(BeadsBitmap);
    Stage.update();

    Stage.addEventListener('click', stageClick);

    $('#clear').click(clear);
    $('#send_data').click(sendData);
    $('#stop').click(sendStopMsg);
    $('#pen').click(toolSelectPen);
    $('#eraser').click(toolSelectEraser);

    for(var index in ColorList){
        $('#color_select').append('<dd data-color-id="' + ColorList[index]['Id'] + '" class="color_dd" style="background-color: ' + ColorList[index]['Color'] + ';"></dd>');
    }

    $('#color_select dd').click(colorSelect);

    drawGrid(); //グリッド描画

    if(!is8x8){
        //29x29モードの場合は画像アップロードフォームと29x29用テンプレートを表示する
        $('#image').show();
        $('#template29').show();
    }

    if(isBeadsDataDefined) drawImage(); //アップロードされた画像を描画

    $('#pen').css({'background': '#87CEFA'});

});



/**
 * ビーズ配置データを初期化
 * @returns {undefined}
 */
function initBeadsData()
{
    BeadsData = [];
    for(var i=0; i<BeadsCount; i++){
        BeadsData.push('0'); //0で無配置状態
    }
}



/**
 * ビーズ色選択
 * @param {type} e
 * @returns {undefined}
 */
function colorSelect(e)
{

    var dd = $(e.target);

    if(dd.prop('id') == 'current_color') return;

    $('#current_color').text('');

    CurrentColor = dd.css('background-color');
    $('#current_color').css('background-color', CurrentColor);

    CurrentColorId = dd.attr('data-color-id');

}



/**
 * ビーズ配置
 * @param {type} e
 * @returns {undefined}
 */
function stageClick(e)
{

    var x = e.stageX;
    var y = e.stageY;

    //8x8モードの場合は描画可能領域外の操作不可
    if(is8x8 && (x < DrawAreaX || x > DrawAreaX + DrawAreaW || y < DrawAreaY || y > DrawAreaY + DrawAreaH)) return;

    var targetX = x - (x % BeadsSize);
    var targetY = y - (y % BeadsSize);

    switch(CurrentTool){
        case ToolList.pen:
            //ペン

            if(CurrentColorId === null) return;

            BeadsBitmapData.clearRect(targetX, targetY, BeadsSize, BeadsSize);

            var BeadsHalfSize = Math.floor(BeadsSize / 2);

            var circle = new createjs.Shape();
            circle.graphics.beginFill(CurrentColor).drawCircle(BeadsHalfSize, BeadsHalfSize, BeadsHalfSize);
            circle.x = 0;
            circle.y = 0;
            circle.cache(0, 0, BeadsSize, BeadsSize);

            var mat = new createjs.Matrix2D(1, 0, 0, 1, targetX, targetY);

            BeadsBitmapData.draw(circle, mat);

            setBeads(Math.floor(targetX / BeadsSize), Math.floor(targetY / BeadsSize), CurrentColorId);
        break;

        case ToolList.eraser:
            //消しゴム

            BeadsBitmapData.clearRect(targetX, targetY, BeadsSize, BeadsSize);

            var BeadsHalfSize = Math.floor(BeadsSize / 2);
            var GridHalfSize = BeadsSize / 5;

            var circle = new createjs.Shape();
            circle.graphics.beginFill(GridColor).drawCircle(BeadsHalfSize, BeadsHalfSize, GridHalfSize);
            circle.x = 0;
            circle.y = 0;
            circle.cache(0, 0, BeadsSize, BeadsSize);

            var mat = new createjs.Matrix2D(1, 0, 0, 1, targetX, targetY);

            BeadsBitmapData.draw(circle, mat);

            setBeads(Math.floor(targetX / BeadsSize), Math.floor(targetY / BeadsSize), 0);
        break;
    }

    Stage.update();

}



/**
 * ビーズセット
 * @param {type} x
 * @param {type} y
 * @param {type} color_id
 * @returns {undefined}
 */
function setBeads(x, y, color_id)
{

	var index = x + y * BeadsStageScale;
	BeadsData[index] = color_id;

}



/**
 * ビーズ配置データをクリア
 * @returns {undefined}
 */
function clear()
{

    if(!confirm('すべてのビーズを削除しますか？')) return;

    initBeadsData();
    drawGrid();

}



/**
 * ペンツールを選択
 * @returns {undefined}
 */
function toolSelectPen()
{
    CurrentTool = ToolList.pen;
    $('#pen').css({'background': '#87CEFA'});
    $('#eraser').css({'background': 'none'});
}



/**
 * 消しゴムツールを選択
 * @returns {undefined}
 */
function toolSelectEraser()
{
    CurrentTool = ToolList.eraser;
    $('#pen').css({'background': 'none'});
    $('#eraser').css({'background': '#87CEFA'});
}



/**
 * ビーズ配置データを送信
 * @returns {undefined}
 */
function sendData()
{

    //ビーズ配置データを文字列に変換
    var data = '';
    for(var index in BeadsData){
        data += BeadsData[index];
    }
    var form = document.createElement('form');
    var request = document.createElement('input');
    document.body.appendChild(form);

    request.name = 'text';
    request.value = data;

    form.appendChild(request);
    form.method = 'POST';
    form.submit();
}

/**
 * 停止命令を送信
 * @returns {undefined}
 */
function sendStopMsg()
{
    var data = 'stop';
    var form = document.createElement('form');
    var request = document.createElement('input');
    document.body.appendChild(form);
    request.name = 'text';
    request.value = data;

    form.appendChild(request);
    form.method = 'POST';
    form.submit();
}


/*
 * ビーズデータ送信成功
 * @param {type} e
 * @returns {undefined}
 */
function onSuccess(e)
{

    alert('データを送信しました。');

}

/*
 * ビーズデータ送信失敗
 * @param {type} e
 * @returns {undefined}
 */
function onError(e)
{

    alert('データを送信中にエラーが発生しました!!!。');

}


function drawImage()
{
    var count = 0;

    for(var y=0; y<BeadsStageScale; y++){
        for(var x=0; x<BeadsStageScale; x++){

            var targetX = x * BeadsSize;
            var targetY = y * BeadsSize;

            var BeadsHalfSize = Math.floor(BeadsSize / 2);

            var circle = new createjs.Shape();
            circle.graphics.beginFill(ColorList[IndexList[ConvertData[count]]].Color).drawCircle(BeadsHalfSize, BeadsHalfSize, BeadsHalfSize);
            circle.x = 0;
            circle.y = 0;
            circle.cache(0, 0, BeadsSize, BeadsSize);

            var mat = new createjs.Matrix2D(1, 0, 0, 1, targetX, targetY);

            BeadsBitmapData.draw(circle, mat);

            setBeads(x, y, ConvertData[count]);

            count ++;

        }
    }


    Stage.update();

}



/*
 * グリッド描画
 * @returns {undefined}
 */
function drawGrid()
{

    BeadsBitmapData.clearRect(0, 0, StageSize, StageSize);

    var count = 0;

    for(var y=0; y<BeadsStageScale; y++){
        for(var x=0; x<BeadsStageScale; x++){

            var targetX = x * BeadsSize;
            var targetY = y * BeadsSize;

            var BeadsHalfSize = Math.floor(BeadsSize / 2);
            var GridHalfSize = BeadsSize / 5;

            var circle = new createjs.Shape();
            circle.graphics.beginFill(GridColor).drawCircle(BeadsHalfSize, BeadsHalfSize, GridHalfSize);
            circle.x = 0;
            circle.y = 0;
            circle.cache(0, 0, BeadsSize, BeadsSize);

            var mat = new createjs.Matrix2D(1, 0, 0, 1, targetX, targetY);

            BeadsBitmapData.draw(circle, mat);

            count ++;

        }
    }


    if(is8x8){
        //8x8領域のみ赤枠で囲む

        var rect = new createjs.Shape();
        rect.graphics.setStrokeStyle(DrawAreaStrokeWidth).beginStroke('#FF0000').drawRect(0, 0, DrawAreaW + DrawAreaStrokeWidth, DrawAreaH + DrawAreaStrokeWidth);
        rect.x = 0;
        rect.y = 0;
        rect.cache(-(DrawAreaStrokeWidth / 2), -(DrawAreaStrokeWidth / 2), DrawAreaW + DrawAreaStrokeWidth * 2, DrawAreaW + DrawAreaStrokeWidth * 2);

        var mat = new createjs.Matrix2D(1, 0, 0, 1, DrawAreaX - DrawAreaStrokeWidth, DrawAreaY - DrawAreaStrokeWidth);

        BeadsBitmapData.draw(rect, mat);

    }


    Stage.update();

}
