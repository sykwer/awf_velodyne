# velodyne_pointcloudの機能追加
feature/t4pb-13375-convert-logicブランチではvelodyne_pointcloudに対し、以下の機能を追加した。
* UDPパケット変換部をOpenMPを用いて実行する機能
* UDPパケット変換部に対する入力パケット群・出力点群をテストベクタとしてファイルに出力する機能

テストベクタのフォーマットに関しては、<https://github.com/tier4/hornet/blob/df/feature/t4pb-12889-convert-logic/kernel/sensing/test_vectors/README.md> を参照。

### 環境設定

以下のノードに対する環境設定について記載する。

#### ノード名：Convert

ファイルへの書き込みを行う場合はConvertノードを起動するlaunchファイルで以下のパラメータ値を設定後に実行する。

|名前|型|説明|既定値|
|:---|:---|:---|:---|
|save_test_vector|bool|UDPパケット変換部に対する入力パケット群・出力点群をファイルへ書き込むかどうかを指定する。<br>True：入力パケット群と出力点群をyaml形式でファイルに書き込む。ファイル名はそれぞれ`convert_test_input.yaml`、`convert_test_output.yaml`。同名のファイルが存在する場合は上書きする。<br>False：ファイルへの書き込みを行わない。|False|

* 設定例
    ```
    def generate_launch_description():
        ...
        launch_arguments = [DeclareLaunchArgument(
            'save_test_vector', default_value='True')]

        return launch.LaunchDescription(launch_arguments + ...)
    ```
