import os


class LatexDocument:
    def __init__(self):

        self.__begin = (
            "\\documentclass[8pt]{article}\n"
            + "\\usepackage{amsmath}\n"
            # + "\n\\usepackage{flexisym}\n"
            # + "\n\\usepackage{breqn}\n"
            + "\\usepackage{geometry}\n"
            + "\\geometry{margin=1cm}\n"
            + "\\begin{document}\n"
        )
        self.__end = "\n\\end{document}"
        self.__sections = {}

    def __make_section(self, title):
        return "\n\\subsection*{" + str(title) + "}\n"

    def add_section(self, section_title, section_latex):
        self.__sections[section_title] = section_latex

    def del_section(self, section_title):
        try:
            del self.__sections[section_title]
        except Exception as e:
            print(e)
            print("Valid Keys:", self.__sections.keys())

    def __replace_and_remove(self, latex_doc):
        remove_strs = [
            "            ",
            "{\\left(t \\right)}",
            "1.0",
        ]

        for r_str in remove_strs:
            latex_doc = latex_doc.replace(r_str, "")

        replace_strs = [("0.5", "\\frac{1}{2}")]

        for old_str, new_str in replace_strs:
            latex_doc = latex_doc.replace(old_str, new_str)

        return latex_doc

    def write_tex(self, file_name=None, output_dir=None):
        sections = ""

        for title, tex in self.__sections.items():
            sections += self.__make_section(title)
            sections += tex

        latex_doc = self.__begin + sections + self.__end
        latex_doc = self.__replace_and_remove(latex_doc)

        if output_dir is None:
            output_dir = "./tex"

        if file_name is None:
            file_name = "out"

        with open(f"{output_dir}/{file_name}.tex", "w") as text_file:
            text_file.write(latex_doc)

        return file_name, output_dir

    def write_pdf(self, file_name=None, output_dir=None):
        file_name, output_dir = self.write_tex(file_name, output_dir)
        os.system(f"pdflatex {output_dir}/{file_name}.tex")
        os.system(f"cp {file_name}.pdf {output_dir}")
        os.system(f"rm {file_name}.aux {file_name}.log {file_name}.pdf")
