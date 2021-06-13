# Build, run, and upload the environment from this makefile
#
# -----------------------------

.PHONY: build
build:
		sudo ./scripts/build.sh

.PHONY: run
run:
		sudo ./scripts/run.sh

.PHONY: dev
dev:
		sudo ./scripts/dev.sh

.PHONY: docs
docs:
		./scripts/docs.sh
