build: clean
	@echo "[MAKE] Building"
	@echo "[MAKE] Copying files"
	@cp src _build/ -r
	@echo "[MAKE] Building strategy files"
	@for file in *.dot; do \
		`cd _build && g2s $$file >> g2s_builds.make.log`; \
	done
	@echo "[MAKE] Done"

push_pi: build
	@echo "[MAKE] Pushing to pi"
	@echo "[TODO]"

clean:
	@echo "[MAKE] Cleaning"
	@if [ -d _build ]; then rm -r _build; fi
