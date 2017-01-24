var jsdom = require("jsdom");

function apply_katex(err, window) {
    var mathElems = window.document.getElementsByClassName("math");
    var katex = require("katex");
    for (key in mathElems) {
        var elem = mathElems[key];
        if (typeof elem == "object" && "innerHTML" in elem) {
            elem.innerHTML = katex.renderToString(elem.innerHTML);
        }
    }
    //window.prettyPrint();
    console.log("<!DOCTYPE html>");
    console.log(window.document.documentElement.outerHTML);
}

jsdom.env(process.argv[2],
        ["https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.3.0/katex.min.js"],
        {
            features: {
                FetchExternalResources: false,//["script"],
                ProcessExternalResources: false//["script"]
            }
        }, function (err, window) {
            var mathElems = window.document.getElementsByClassName("math");
            for (key in mathElems) {
                var elem = mathElems[key];
                if (typeof elem == "object" && "innerHTML" in elem) {
                    if (elem.tagName == 'DIV') {
                        elem.innerHTML = ("\n\n\\\\[\n"
                            + elem.innerHTML + "\n\\\\]\n\n");
                    } else {
                        elem.innerHTML = "$" + elem.innerHTML + "$";
                    }
                }
            }
            var paragraphs = window.document.getElementsByTagName("p");
            for (key in paragraphs) {
                var elem = paragraphs[key];
                if (typeof elem == "object" && "textContent" in elem
                        && elem.textContent == "" && "parentNode" in elem) {
                    elem.parentNode.removeChild(elem);
                }
            }
            var scripts = window.document.getElementsByTagName("script");
            for (key in scripts) {
                var elem = scripts[key];
                if (typeof elem == "object" && "parentNode" in elem) {
                    elem.parentNode.removeChild(elem);
                }
            }
            var images = window.document.getElementsByTagName("img");
            for (key in images) {
                var elem = images[key];
                if (typeof elem == "object" && "src" in elem) {
                    elem.src = elem.src.replace(".png", "e.png");
                }
            }
            var cppElems = window.document.getElementsByClassName("lang-cpp");
            for (key in cppElems) {
                var elem = cppElems[key];
                if (typeof elem == "object" && "className" in elem) {
                    elem.className += " cpp";
                }
            }
            var seealsoElems = window.document.getElementsByClassName("seealso");
            for (key in seealsoElems) {
                var elem = seealsoElems[key];
                if (typeof elem == "object" && "textContent" in elem) {
                    elem.innerHTML = "$\\text{\\small " + elem.textContent + "}$";
                }
            }
            console.log("<!DOCTYPE html>");
            console.log(window.document.documentElement.outerHTML);
        });
