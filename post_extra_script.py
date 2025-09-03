Import("env")

elf_path = env.subst("$BUILD_DIR/${PROGNAME}.elf")
hex_path = elf_path.replace(".elf", ".hex")

# パスをダブルクオートで囲うことでスペースを回避
env.AddPostAction(
    elf_path,
    env.VerboseAction(
        f'"arm-none-eabi-objcopy" -O ihex "{elf_path}" "{hex_path}"',
        "Building HEX from ELF"
    )
)
