obj-y		:= hermit.o mmnif.o
clean-files	:= boot.h boot.bin

$(obj)/hermit.o: $(obj)/boot.h

$(obj)/boot.bin: $(src)/boot.asm
	@$(kecho) "  NASM    $@"
	$Qnasm -fbin -o $@ $<

$(obj)/boot.h: $(obj)/boot.bin
	@$(kecho) "  HEXDUMP $@"
	$Qecho "static const uint8_t boot_code[] = { \
		$(shell hexdump -v -e '"0x" 1/1 "%02X" ", "' $<) \
	};" > $@