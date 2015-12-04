(TeX-add-style-hook
 "preamble"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("fontenc" "T1") ("geometry" "a4paper" "top=1in" "bottom=1.1in" "left=1in" "right=1in") ("appendix" "toc" "page") ("inputenc" "utf8")))
   (add-to-list 'LaTeX-verbatim-environments-local "lstlisting")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "lstinline")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "lstinline")
   (TeX-run-style-hooks
    "fontenc"
    "geometry"
    "appendix"
    "inputenc"
    "amsmath"
    "attachfile"
    "caption"
    "commath"
    "graphicx"
    "hyperref"
    "listings"
    "mathtools"
    "siunitx"
    "subcaption"
    "url"
    "varioref"
    "wrapfig"
    "xcolor")
   (TeX-add-symbols
    '("di" 2)
    "YAMLcolonstyle"
    "YAMLkeystyle"
    "YAMLvaluestyle"
    "language"
    "ProcessThreeDashes")
   (LaTeX-add-caption-DeclareCaptions
    '("\\DeclareCaptionFormat{myformat}" "Format" "myformat"))))

