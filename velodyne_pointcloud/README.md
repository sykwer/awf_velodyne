# velodyne_pointcloudの機能追加
feature/t4pb-12889-convert-logicブランチではvelodyne_pointcloudに対し、以下の機能を追加した。
* VLP-16およびVLS-128のUDPパケット変換部をOpenMPを用いて実行する機能
* UDPパケット変換部に対する入力パケット群・出力点群（距離判定による抽出前後）をテストベクタとしてファイルに出力する機能

### 環境設定

以下のノードに対する環境設定について記載する。

#### ノード名：Convert

ファイルへの書き込みを行う場合はConvertノードを起動するlaunchファイルで以下のパラメータ値を設定後に実行する。

|パラメータ名|型|説明|既定値|
|:---|:---|:---|:---|
|save_test_vector|bool|入力点群・出力点群をファイルへ書き込むかどうかを指定する。<br>True：入力点群と出力点群をyaml形式でファイルに書き込む。入力点群ファイル名は`[名前空間]_[node名]_input_vector.yaml`、距離判定による抽出前の出力点群ファイル名は`[名前空間]_[node名]_output_vector.yaml`、距離判定による抽出後の出力点群ファイル名は`[名前空間]_[node名]_extracted_output_vector.yaml`。同名のファイルが存在する場合は上書きする。<br>False：ファイルへの書き込みを行わない。|False|
|test_vector_sampling_start|unsigned int|入力点群・出力点群のサンプリングを始めるインデックスを指定する（値は0始まり）。|0|
|test_vector_sampling_rate|unsigned int|入力点群・出力点群のサンプリング頻度を指定する。|0|
|test_vector_sampling_end|unsigned int|入力点群・出力点群のサンプリングを終わるインデックスを指定する（値は0始まり）。このパラメータで設定したインデックスの入力点群を受信した時、ファイルに書き込む。|0|

* 設定例（0番目から199番目の入力点群受信のうち、20回ごとに入力点群・出力点群をテストベクタとしてファイルに出力する）
    ```
    def generate_launch_description():
        ...
        launch_arguments = []
        launch_arguments.append(DeclareLaunchArgument('save_test_vector', default_value='True'))
        launch_arguments.append(DeclareLaunchArgument('test_vector_sampling_start', default_value='0'))
        launch_arguments.append(DeclareLaunchArgument('test_vector_sampling_rate', default_value='20'))
        launch_arguments.append(DeclareLaunchArgument('test_vector_sampling_end', default_value='199'))
        ...

        return launch.LaunchDescription(launch_arguments + ...)
    ```
    
### テストベクタのフォーマット

#### 入力テストベクタ

* フレームの配列

    各要素が1フレームを表すトップレベルの配列。各要素は以下のフィールドを持つ。

    * フレームIDのハッシュ

        キー：`frame_id`、値：`(フレームID)`を持つハッシュ。

    * パケット群のハッシュ

        キー：`packets`、値：`(パケットの配列)`を持つハッシュ。

* パケットの配列

    各要素が1パケットを表す配列。各要素は以下のフィールドを持つ。

    * パケットIDのハッシュ

        キー：`packet_id`、値：`(パケットID)`を持つハッシュ。

    * パケットデータのハッシュ

        キー：`data`、値：`(パケットデータの配列)`を持つハッシュ。

* パケットデータの配列

    各要素が1バイトデータを表す配列。要素数は1206。

### Example

```
- frame_id: 0
  packets:
    - packet_id: 0
      data: [255, ..., 34]
      ...
    - packet_id: 74
      data: [255, ..., 34]
...
- frame_id: 18
  packets:
    - packet_id: 0
      data: [255, ..., 34]
      ...
    - packet_id: 73
      data: [255, ..., 34]
```

#### 出力テストベクタ（距離判定による抽出前後共通）

* フレームの配列

    各要素が1フレームを表すトップレベルの配列。各要素は以下のフィールドを持つ。

    * フレームIDのハッシュ

        キー：`frame_id`、値：`(フレームID)`を持つハッシュ。

    * 点群のハッシュ

        キー：`clouds`、値：`(点データの配列)`を持つハッシュ。

* 点データの配列

    各要素が1つの点を表す配列。要素数は9。各要素は順に以下のデータを表す。

    * x座標
    * y座標
    * z座標
    * return_type
    * ring
    * azimuth
    * distance
    * intensity
    * time_stamp

### Example

```
- frame_id: 0
  clouds:
    - [2.028318, -0.02265747, -0.54352, 27, "\x01", 0, 64, 2.1, 5.5296e-05]
    ...
    - [2.302737, 0.1158461, -0.04024525, 16, "\x01", 7, 35712, 2.306, 0.001304064]
...
- frame_id: 18
  clouds:
    - [2.028384, -0.01557718, -0.54352, 27, "\x01", 0, 44, 2.1, 0]
    ...
    - [2.456204, 0.1235667, 0.5677765, 16, "\x01", 14, 35712, 2.524, 0.00130176]
```
