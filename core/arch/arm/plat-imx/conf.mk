PLATFORM_FLAVOR ?= mx6ulevk

# Get SoC associated with the PLATFORM_FLAVOR
mx6ul-flavorlist = \
	mx6ulevk \

mx6ull-flavorlist = \
	mx6ullevk \

mx6q-flavorlist = \
	mx6qsabrelite \
	mx6qsabresd \
	mx6qhmbedge \
	mx6qvab820 \

mx6sx-flavorlist = \
	mx6sxsabreauto \
	mx6sxudooneofull \

mx6d-flavorlist = \
	mx6dhmbedge \

mx6dl-flavorlist = \
	mx6dlsabresd \
	mx6dlhmbedge \

mx6s-flavorlist = \
	mx6shmbedge \

mx7-flavorlist = \
	mx7dsabresd \
	mx7swarp7 \
	mx7dclsom \

ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx6ul-flavorlist)))
$(call force,CFG_MX6,y)
$(call force,CFG_MX6UL,y)
$(call force,CFG_TEE_CORE_NB_CORE,1)
include core/arch/arm/cpu/cortex-a7.mk
else ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx6ull-flavorlist)))
$(call force,CFG_MX6,y)
$(call force,CFG_MX6ULL,y)
$(call force,CFG_TEE_CORE_NB_CORE,1)
include core/arch/arm/cpu/cortex-a7.mk
else ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx6q-flavorlist)))
$(call force,CFG_MX6,y)
$(call force,CFG_MX6Q,y)
$(call force,CFG_TEE_CORE_NB_CORE,4)
else ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx6d-flavorlist)))
$(call force,CFG_MX6,y)
$(call force,CFG_MX6D,y)
$(call force,CFG_TEE_CORE_NB_CORE,2)
else ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx6dl-flavorlist)))
$(call force,CFG_MX6,y)
$(call force,CFG_MX6DL,y)
$(call force,CFG_TEE_CORE_NB_CORE,2)
else ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx6s-flavorlist)))
$(call force,CFG_MX6,y)
$(call force,CFG_MX6S,y)
$(call force,CFG_TEE_CORE_NB_CORE,1)
else ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx6sx-flavorlist)))
$(call force,CFG_MX6,y)
$(call force,CFG_MX6SX,y)
$(call force,CFG_TEE_CORE_NB_CORE,1)
else ifneq (,$(filter $(PLATFORM_FLAVOR),$(mx7-flavorlist)))
$(call force,CFG_MX7,y)
CFG_TEE_CORE_NB_CORE ?= 2
include core/arch/arm/cpu/cortex-a7.mk
else
$(error Unsupported PLATFORM_FLAVOR "$(PLATFORM_FLAVOR)")
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx7dsabresd))
CFG_DDR_SIZE ?= 0x40000000
CFG_NS_ENTRY_ADDR ?= 0x80800000
$(call force,CFG_TEE_CORE_NB_CORE,2)
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx7dclsom))
CFG_DDR_SIZE ?= 0x40000000
CFG_UART_BASE ?= UART1_BASE
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx7swarp7))
CFG_DDR_SIZE ?= 0x20000000
CFG_NS_ENTRY_ADDR ?= 0x80800000
CFG_BOOT_SECONDARY_REQUEST ?= n
$(call force,CFG_TEE_CORE_NB_CORE,1)
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx6qsabresd mx6dlsabresd \
	mx6dlsabrelite mx6dhmbedge mx6dlhmbedge))
CFG_DDR_SIZE ?= 0x40000000
CFG_NS_ENTRY_ADDR ?= 0x12000000
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx6qhmbedge))
CFG_DDR_SIZE ?= 0x80000000
CFG_UART_BASE ?= UART1_BASE
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx6qvab820))
CFG_DDR_SIZE ?= 0x40000000
CFG_UART_BASE ?= UART2_BASE
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx6shmbedge))
CFG_DDR_SIZE ?= 0x40000000
CFG_NS_ENTRY_ADDR ?= 0x12000000
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx6qsabrelite mx6dlsabrelite))
CFG_DDR_SIZE ?= 0x40000000
CFG_NS_ENTRY_ADDR ?= 0x12000000
CFG_UART_BASE ?= UART2_BASE
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx6sxsabreauto))
CFG_DDR_SIZE ?= 0x80000000
CFG_NS_ENTRY_ADDR ?= 0x80800000
endif

ifeq ($(PLATFORM_FLAVOR), mx6sxudooneofull)
CFG_DDR_SIZE ?= 0x40000000
CFG_UART_BASE ?= UART1_BASE
endif

ifneq (,$(filter $(PLATFORM_FLAVOR),mx6ulevk mx6ullevk))
CFG_DDR_SIZE ?= 0x20000000
CFG_NS_ENTRY_ADDR ?= 0x80800000
endif

# i.MX6 Solo/SoloX/DualLite/Dual/Quad specific config
ifeq ($(filter y, $(CFG_MX6Q) $(CFG_MX6D) $(CFG_MX6DL) $(CFG_MX6S) \
      $(CFG_MX6SX)), y)
include core/arch/arm/cpu/cortex-a9.mk

$(call force,CFG_PL310,y)

CFG_PL310_LOCKED ?= y
CFG_ENABLE_SCTLR_RR ?= y
endif

ifeq ($(filter y, $(CFG_MX6Q) $(CFG_MX6D) $(CFG_MX6DL) $(CFG_MX6S)), y)
CFG_DRAM_BASE ?= 0x10000000
endif

ifneq (,$(filter y, $(CFG_MX6UL) $(CFG_MX6ULL) $(CFG_MX6SX)))
CFG_DRAM_BASE ?= 0x80000000
endif

ifeq ($(filter y, $(CFG_MX7)), y)
CFG_INIT_CNTVOFF ?= y
CFG_DRAM_BASE ?= 0x80000000
endif

ifneq (,$(filter y, $(CFG_MX6) $(CFG_MX7)))
$(call force,CFG_GENERIC_BOOT,y)
$(call force,CFG_GIC,y)
$(call force,CFG_IMX_UART,y)
$(call force,CFG_PM_STUBS,y)
ifneq ($(filter y, $(CFG_FSL_SEC)), y)
$(call force,CFG_WITH_SOFTWARE_PRNG,y)
endif

CFG_BOOT_SYNC_CPU ?= n
CFG_BOOT_SECONDARY_REQUEST ?= y
CFG_CRYPTO_SIZE_OPTIMIZATION ?= n
CFG_DT ?= y
CFG_PAGEABLE_ADDR ?= 0
CFG_PSCI_ARM32 ?= y
CFG_SECURE_TIME_SOURCE_REE ?= y
CFG_UART_BASE ?= UART1_BASE
CFG_WITH_STACK_CANARIES ?= y

CFG_TZDRAM_START ?= ($(CFG_DRAM_BASE) - 0x02000000 + $(CFG_DDR_SIZE))
CFG_TZDRAM_SIZE ?= 0x01e00000
CFG_SHMEM_START ?= ($(CFG_TZDRAM_START) + $(CFG_TZDRAM_SIZE))
CFG_SHMEM_SIZE ?= 0x00200000

ta-targets = ta_arm32
endif

ifeq ($(filter y, $(CFG_PSCI_ARM32)), y)
CFG_HWSUPP_MEM_PERM_WXN = n
CFG_IMX_WDOG ?= y
endif

CFG_MMAP_REGIONS ?= 24
