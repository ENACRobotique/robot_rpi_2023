g2sCompileFiles := strategy.py
buildRemoveFiles := strategy.dot

build: clean
	@echo "[MAKE] Building"
	@echo "[MAKE] Copying files"
	@cp src _build/ -r
	@echo "[MAKE] Building strategy"
	@for file in $(g2sCompileFiles); do \
		echo "[MAKE] \\ Compiling $$file"; \
		`cd _build && g2sCompiler $$file -o $$file > $$file.make.log`; \
	done
	@echo "[MAKE] Removing build files"
	@for file in $(buildRemoveFiles); do \
		echo "[MAKE] \\ Removing $$file"; \
		`cd _build && rm $$file`; \
	done

push_pi:
	@echo "[MAKE] Pushing to pi"
	@echo "[TODO]"

clean:
	@echo "[MAKE] Cleaning"
	@if [ -d _build ]; then rm -r _build; fi
