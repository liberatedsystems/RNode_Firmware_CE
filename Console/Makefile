PATH_RETICULUM_WEBSITE=../../sites/reticulum.network
PATH_PACKAGES=../../dist_archive

clean:
	@echo Cleaning...
	@-rm -rf ./build

dirs:
	@mkdir -p ./build
	@mkdir -p ./build/3d
	@mkdir -p ./build/pkg
	@mkdir -p ./build/css
	@mkdir -p ./build/gfx
	@mkdir -p ./build/images

pages:
	python3 ./build.py

pages-debug:
	python3 ./build.py --no-gz --no-remap

sourcepack:
	@echo Packing firmware sources...
	zip --junk-paths -r build/pkg/rnode_firmware.zip ../arduino-cli.yaml ../Bluetooth.h ../Boards.h ../Config.h ../Console.h ../Device.h ../Display.h ../Framing.h ../Graphics.h ../Input.h ../LICENSE ../Makefile ../MD5.cpp ../MD5.h ../Modem.h ../partition_hashes ../Power.h ../README.md ../release_hashes.py ../RNode_Firmware_CE.ino ../ROM.h ../sx126x.cpp ../sx126x.h ../sx127x.cpp ../sx127x.h ../sx128x.cpp ../sx128x.h ../Utilities.h
	
data:
	@echo Including assets...
	@cp assets/css/* build/css/
	@cp assets/gfx/* build/gfx/
	@cp assets/images/* build/images/
	@cp assets/stl/* build/3d/
	#@cp assets/pkg/* build/pkg/
	# @cp assets/scripts/* build/scripts/
	# @cp -r ../../Reticulum/docs/manual/* build/reticulum_manual/
	# @cp -r ../../Reticulum/docs/Reticulum\ Manual.pdf build/reticulum_manual/

external:
	make -C $(PATH_RETICULUM_WEBSITE) clean website
	-rm -r $(PATH_PACKAGES)/reticulum.network
	cp -r $(PATH_RETICULUM_WEBSITE)/build $(PATH_PACKAGES)/reticulum.network

site: clean external dirs data sourcepack pages

local: clean external dirs data sourcepack pages-debug

serve:
	python3 -m http.server 7777 --bind 127.0.0.1 --directory ./build
