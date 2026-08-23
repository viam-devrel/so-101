
GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/arm

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo	
	MODULE_BINARY = bin/arm.exe
endif

GO_SRC         := $(wildcard *.go) $(shell find cmd internal components services -name '*.go' 2>/dev/null)
EMBEDS         := $(shell find internal/geometry -type f ! -name '*.go' 2>/dev/null)
RUNTIME_ASSETS := $(shell find assets -type f 2>/dev/null)
$(if $(GO_SRC),,$(error GO_SRC is empty — has the source layout moved?))
$(if $(RUNTIME_ASSETS),,$(error RUNTIME_ASSETS is empty — has assets/ moved?))

$(MODULE_BINARY): Makefile go.mod $(GO_SRC) $(EMBEDS)
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) ./cmd/module

lint:
	gofmt -s -w .

build-app:
	cd setup-app && pnpm run build && cd ..

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY) first_run.sh build-app $(RUNTIME_ASSETS)
ifeq ($(VIAM_TARGET_OS), windows)
	jq '.entrypoint = "./bin/arm.exe"' meta.json > temp.json && mv temp.json meta.json
else
	strip $(MODULE_BINARY)
endif
	tar --exclude='.DS_Store' -czf $@ meta.json first_run.sh $(MODULE_BINARY) setup-app/build/ assets/
ifeq ($(VIAM_TARGET_OS), windows)
	git checkout meta.json
endif

module: test module.tar.gz

all: test module.tar.gz

setup:
	go mod tidy
	cd setup-app && pnpm install
