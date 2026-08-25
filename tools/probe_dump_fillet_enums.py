import sys
import pythoncom
import win32com.client

LOG_PATH = r"C:\Users\hide2\AppData\Local\Temp\claude\C--Users-hide2-IdeaBox-PartMaker\7974b216-26f6-44d9-aae1-a7fcf52753e0\scratchpad\dump_enums2.log"
log = open(LOG_PATH, "w", encoding="utf-8", buffering=1)


def p(*args):
    msg = " ".join(str(a) for a in args)
    log.write(msg + "\n")
    log.flush()
    print(msg)


def dump_enum_from_typeinfo(ti, label):
    attr = ti.GetTypeAttr()
    p(f"  [{label}] typekind={attr[5]} nvars={attr[7]}")
    if attr[5] != pythoncom.TKIND_ENUM:
        p(f"    not an enum, skipping")
        return
    for vi in range(attr[7]):
        vd = ti.GetVarDesc(vi)
        memid = vd[0]
        names = ti.GetNames(memid)
        name = names[0] if names else "?"
        p(f"    {name} raw_vd={vd!r}")


sys.path.insert(0, r"C:\Users\hide2\IdeaBox\PartMaker\synthetic_generator\src")
from synthetic_generator.gsd_build import SyntheticPartBuilder

builder = SyntheticPartBuilder()
doc = builder.new_part_document()
part = doc.Part
hsf = part.HybridShapeFactory
sf = part.ShapeFactory
body = part.HybridBodies.Add()

# build a trivial shape + fillet feature just so we get a typed FilletFeature object
face = builder.rect_fill(hsf, part, body, [(0, 0, 0), (0, 60, 0), (60, 60, 10), (60, 0, 10)])
part.Update()

f = sf.AddNewSurfaceEdgeFilletWithConstantRadius(None, 0, 2.0)
p("constructed fillet feature, dumping its typeinfo funcdescs for property names of interest")

ti = f._oleobj_.GetTypeInfo()
attr = ti.GetTypeAttr()
nfuncs = attr[6]
p(f"fillet feature typeinfo: nfuncs={nfuncs}")

targets = {"EdgePropagation", "FilletBoundaryRelimitation", "FilletTrimSupport"}
for fi in range(nfuncs):
    try:
        fd = ti.GetFuncDesc(fi)
        memid = fd[0]
        names = ti.GetNames(memid)
        name = names[0] if names else None
        if name not in targets:
            continue
        invkind = fd[4]  # INVOKEKIND
        # fd[8] = elemdescFunc -> return type; fd[2] = args
        elemdesc_ret = fd[8]
        typedesc = elemdesc_ret[0]
        p(f"func '{name}' invkind={invkind} typedesc={typedesc}")
        # typedesc: (vt, ...) if vt == VT_USERDEFINED(29), typedesc[1] is href
        vt = typedesc[0]
        if vt == pythoncom.VT_USERDEFINED:
            href = typedesc[1]
            ref_ti = ti.GetRefTypeInfo(href)
            dump_enum_from_typeinfo(ref_ti, name)
        elif vt == pythoncom.VT_PTR:
            inner = typedesc[1]
            p(f"  (pointer) inner typedesc={inner}")
            if inner[0] == pythoncom.VT_USERDEFINED:
                href = inner[1]
                ref_ti = ti.GetRefTypeInfo(href)
                dump_enum_from_typeinfo(ref_ti, name)
        else:
            p(f"  vt={vt} not user-defined, no enum to dump")
    except Exception as exc:
        p(f"  [error at fi={fi}] {exc}")

doc.Close()
p("DONE")
