

api_src := $(shell pwd)/../rtl836x
api := API_Source

api_files := $(wildcard $(api_src)/$(api)/*.c)
api_files := $(subst $(api_src)/,,$(api_files))
api_files := $(patsubst %.c,%.o,$(api_files))

EXTRA_CFLAGS += -I$(api_src)/$(api)

obj-$(CONFIG_RTL836x) += rtl836x.o

$(info $(api_files) $(EXTRA_CFLAGS))

rtl836x-objs := $(api_files) rtl8363.o rtl8367s_dbg.o




