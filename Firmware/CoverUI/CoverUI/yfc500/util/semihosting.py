Import("env")

# Add semihosting feature
env.Append(
    LINKFLAGS=["--specs=rdimon.specs",
               "-Wl,-u,_printf_float",
               "-Wl,-u,_printf_scanf"],
    LIBS=["rdimon"]
)
