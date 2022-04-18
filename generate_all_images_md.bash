#!/bin/bash

TITLE="all-images"
OUTPUT="# ${TITLE}\n\n"

DIRS=($(ls -d */))

# ディレクトリへのリンク集
for directory in ${DIRS[@]}; do
    OUTPUT+="- [${directory%/}](#${directory%/})\n"
done

OUTPUT+="\n"

for directory in ${DIRS[@]}; do
    OUTPUT+="## ${directory%/}\n\n"

    FILES=($(find ${directory} -type f))
    for filename in ${FILES[@]}; do
        # 画像ファイルへのリンクと画像を表示
        OUTPUT+="- [${filename##*/}](${filename})\n\n"
        OUTPUT+="![${filename##*/}](${filename})\n\n"
    done

    OUTPUT+="[back to top](#${TITLE})\n\n"
    OUTPUT+="---\n\n"
done

# 改行を表示
echo -e ${OUTPUT} > all-images.md

# CC-BYを追記
echo '<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />The files in this page are licensed under the <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.' >> all-images.md
echo "" >> all-images.md
echo "(C) 2016 RT Coporation \<support@rt-net.jp\>" >> all-images.md