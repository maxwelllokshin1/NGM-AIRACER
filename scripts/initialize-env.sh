#!/bin/bash

# Copies .env.example to .env on the host machine if .env does not already exist.

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ENV_FILE="${SCRIPT_DIR}/../.env"
EXAMPLE_FILE="${SCRIPT_DIR}/../.env.example"

if [ -f "${ENV_FILE}" ]; then
    echo ".env already exists. Skipping."
    exit 0
fi

if [ ! -f "${EXAMPLE_FILE}" ]; then
    echo "ERROR: .env.example not found at ${EXAMPLE_FILE}"
    exit 1
fi

cp "${EXAMPLE_FILE}" "${ENV_FILE}"
echo "Created .env from .env.example"
echo "Edit .env to set your build target, display config, and Onshape credentials."
