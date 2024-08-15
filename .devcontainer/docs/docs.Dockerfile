FROM squidfunk/mkdocs-material:9

# get docs requirements file
COPY ./docs/requirements.txt /requirements.txt

# install requirements
RUN pip install -r /requirements.txt
