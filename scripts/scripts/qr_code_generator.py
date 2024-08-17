#! /usr/bin/env python
import qrcode 
img_name = "medium.png"  
def generate(data="shelf1-1", img_name="shelf1-1.png"):
     img = qrcode.make(data) #generate QRcode     
     img.save(img_name)     
     return img  
generate()