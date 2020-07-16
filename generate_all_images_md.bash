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

    FILES=($(ls ${directory}))
    for filename in ${FILES[@]}; do
        # 画像ファイルへのリンクと画像を表示
        OUTPUT+="- [${filename}](${directory}${filename})\n\n"
        OUTPUT+="![${filename}](${directory}${filename})\n\n"
    done

    OUTPUT+="[back to top](#${TITLE})\n\n"
    OUTPUT+="---\n\n"
done

# 改行を表示
echo -e ${OUTPUT} > all-images.md
