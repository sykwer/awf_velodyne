# velodyne_pointcloudの機能追加
feature/t4pb-12889-convert-logicブランチではvelodyne_pointcloudに対し、以下の機能を追加した。
* VLP-16およびVLS-128のUDPパケット変換部をOpenMPを用いて実行する機能
* UDPパケット変換部に対する入力パケット群・出力点群をテストベクタとしてファイルに出力する機能

テストベクタのフォーマットに関しては、<https://github.com/tier4/hornet/blob/df/feature/t4pb-12889-convert-logic/kernel/sensing/test_vectors/README.md> を参照。

### 環境設定

以下のノードに対する環境設定について記載する。

#### ノード名：Convert

ファイルへの書き込みを行う場合はConvertノードを起動するlaunchファイルで以下のパラメータ値を設定後に実行する。

|パラメータ名|型|説明|既定値|
|:---|:---|:---|:---|
|save_test_vector|bool|入力点群・出力点群をファイルへ書き込むかどうかを指定する。<br>True：入力点群と出力点群をyaml形式でファイルに書き込む。ファイル名はそれぞれ`[名前空間]_[node名]_input_vector.yaml`、`[名前空間]_[node名]_output_vector.yaml`。同名のファイルが存在する場合は上書きする。<br>False：ファイルへの書き込みを行わない。|False|
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
