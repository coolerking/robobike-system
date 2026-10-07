FROM pytorch/pytorch:latest

WORKDIR /workspace
COPY ml/visualization/requirements.txt /tmp/visualization-requirements.txt
COPY ml/data_processing/requirements.txt /tmp/data-processing-requirements.txt
COPY ml/training/requirements.txt /tmp/training-requirements.txt
RUN python -m pip install --no-cache-dir \
    -r /tmp/visualization-requirements.txt \
    -r /tmp/data-processing-requirements.txt \
    -r /tmp/training-requirements.txt

CMD ["sleep", "infinity"]
