build: clean
	@echo "[MAKE] Building"
	@echo "[MAKE] Copying files"
	@cp src _build/ -r
	@echo "[MAKE] Building strategy files"
	@`cd _build && g2s *.dot >> g2s_builds.make.log`
	@echo "[MAKE] Done"

push_pi: build
	@echo "[MAKE] Pushing to pi"
	@echo "[TODO]"

clean:
	@echo "[MAKE] Cleaning"
	@if [ -d _build ]; then rm -r _build; fi
