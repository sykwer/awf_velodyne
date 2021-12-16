# velodyne_pointcloudの機能追加
feature/t4pb-13559-test-vectorブランチではvelodyne_pointcloudに対し、以下の機能を追加した。
* fix distortion(interpolate)に対する入力点群・出力点群をテストベクタとしてファイルに出力する機能

### 環境設定

以下のノードに対する環境設定について記載する。

#### ノード名：Interpolate

テストベクタをファイルへ書き込む場合はInterpolateノードを起動する前にlaunchファイルで以下のパラメータ値を設定後に実行する。

|パラメータ名|型|説明|既定値|
|:---|:---|:---|:---|
|save_test_vector|bool|fix distortionに対する入力点群・出力点群をファイルへ書き込むかどうかを指定する。<br>True：入力点群と出力点群をyaml形式でファイルに書き込む。ファイル名はそれぞれ`[名前空間]_[node名]_input_vector.yaml`、`[名前空間]_[node名]_output_vector.yaml`。同名のファイルが存在する場合は上書きする。<br>False：ファイルへの書き込みを行わない。|False|
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

#### 入力・出力共通

* フレームの配列

    各要素が1フレームを表すトップレベルの配列。各要素は以下のフィールドを持つ。

    * フレームIDのハッシュ

        キー：`frame_id`、値：`(フレームID)`を持つハッシュ。

    * 点群のハッシュ

        キー：`cloud`、値：`(点群情報のハッシュ群)`を持つハッシュ。

* 点群情報のハッシュ群

    `pcl::pointcloud<velodyne_pointcloud::PointXYZIRADT>`が持つ情報を格納したハッシュ群。以下のフィールドを持つ。

    * `points`要素配列のハッシュ

        キー：`points`、値：`(points要素配列)`を持つハッシュ。

    * `width`要素のハッシュ

        キー：`width`、値：`(width要素)`を持つハッシュ。
    
    * `height`要素のハッシュ

        キー：`height`、値：`(height要素)`を持つハッシュ。

    * `is_dense`要素のハッシュ

        キー：`is_dense`、値：`(is_dense要素)`を持つハッシュ。

* points要素配列

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
  cloud:
    points:
      - [0.9598088, 1.769561, -0.01282597, 27, "\x01", 0, 58, 2.1, 1585897258.110364]
      ...
      - [7.348701, -6.598307, 11.98354, 1, "\x01", 14, 15139, 14.082, 1585897258.152284]
    width: 5771
    height: 1
    is_dense: true
...
- frame_id: 9
  cloud:
    points:
      - [-22.95948, -0.06209081, 6.591853, 2, "\x01", 11, 26044, 24.376, 1585897281.282459]
      ...
      - [1.008074, 4.119726, 0.01999664, 4, "\x01", 15, 36, 3.922, 1585897281.410331]
    width: 19237
    height: 1
    is_dense: true
```
