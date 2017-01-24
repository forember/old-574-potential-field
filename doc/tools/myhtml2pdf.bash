#!/bin/bash

if [ $# -ne 2 ]; then
    echo "myhtml2pdf.bash <html> <pdf>"
    exit
fi

htmlFile="$1"
pdfFile="$2"
tmpFile="$(dirname "$0")/../.tmp.html"
styleFile="$(dirname "$0")/pdfLatexHeader.tex"

quirksOff='<script>
document.compatMode = "CSS1Compat";
</script>'

titleLineNo="$(sed -n '/\<title\>/=' "$htmlFile")"

processHTML () {
    head "-n$titleLineNo" "$htmlFile"
    echo "$quirksOff"
    tail "-n+$((titleLineNo + 1))" "$htmlFile"
}

processHTML >"$tmpFile"
sed -i 's/katex\.render/\/\/katex\.render/g' "$tmpFile"
nodejs "$(dirname "$0")/applykatex.js" "$tmpFile" \
    | sed 's/file:\/\///g' \
    | pandoc --latex-engine=xelatex -V geometry:margin=1in \
        -V author:'Chris McKinney' -H "$styleFile" \
        -f html+tex_math_double_backslash+tex_math_dollars+raw_tex \
        -o "$pdfFile"

conversionStatus="$?"
rm "$tmpFile"

exit "$conversionStatus"
