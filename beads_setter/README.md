# Install

```shell
$ pip install flask
```



# Test

以下のコマンドを実行後、[localhost:5000](localhost:5000)にアクセスする。

```shell
$ python beads_setter.py
```

プログラムを終了したいときは[localhost:5000/stop](localhost:5000/stop)にアクセスする。



# DataFormat

29x29の盤面を想定し、入力したビーズの情報をString型のROSMessageとして送信している。色の情報についてはbeads_setter.jsを参照してください。



# Tips

もしjsファイルの更新が反映されない場合、以下のコマンドを実行すると直る場合がある。

```shell
Windows : Ctrl + Fn + F5

Mac : Cmd + Shift + R

Linux : Cmd + Shift + Rｚ
```