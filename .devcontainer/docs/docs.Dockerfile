FROM squidfunk/mkdocs-material

# get docs requirements file
COPY ./docs/requirements.txt /requirements.txt

# install requirements
RUN pip install -Ur /requirements.txt
