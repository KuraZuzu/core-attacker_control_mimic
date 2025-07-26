# VScode+CMakeでのNucleoF4への書き込み

### ST-LinkでNucleoF4ボードに書き込む
- ビルドターゲット`BUILD_ONLY`でビルド（`.bin`及び`.hex`を生成する）のみを行う
- ビルドターゲット`BUILD_AND_WRITE`でビルドし、これをマイコンに対して書き込みを行う

### VScodeでの実行エラーについて
例えば、VScodeで実行すると以下のようなエラーが出るが、これはVScode自体の設定によるものなので書き込みは出来る（ただし、実行の必要性は無い）。

```bash:実行エラー
bash: /home/<NucleoF4のプロジェクト>/build/NucleoF4_For_VScode.elf: バイナリファイルを実行できません: 実行形式エラー
``````

<br>

VScodeでの拡張機能`CMake Tools`をインストールして使用した場合
- VScodeではターゲット（`BUILD_ONLY`,`BUILD_AND_WRITE`）を選択して`Build`ボタンで完結する
- VScodeの例では実行ボタンは必要ない
