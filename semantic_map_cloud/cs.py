#!/usr/bin/env python
# coding=utf8
# author=evi1m0@2015<ff0000team>

import sys
import time
import requests
import json
import base64
import os,sys
import subprocess

#http://api.cloudsightapi.com/image_requests?Authorization=CloudSight 8zTOpqEh-VO4eUa_eZS5TQ&image=/home/chelsea/Desktop/cloudsight/1.png&locale=zh-CN&language=zh-CN

def _api(url):
    command = 'curl -i -X POST -H "Authorization: CloudSight 0CsE_GXki6BSdqqcOfzO1g" -F "image_request[image]=@%s" -F "image_request[locale]=en-US" https://api.cloudsightapi.com/image_requests' % url
    
    child1 = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    
    #os.system(command)
    token_json = child1.stdout.read()
    #print("*******************")
    #print(token_json)
    token = token_json.split('"token":')[1].split('"')[1]
    #print(token)

    count = 1
    res_url = 'http://api.cloudsightapi.com/image_responses/'
    headers = {
                'Origin': 'http://cloudsightapi.com',
                'HOST': 'api.cloudsightapi.com',
                #'Authorization': 'CloudSight amZd_zG32VK-AoSz05JLIA',
		'Authorization': 'CloudSight 0CsE_GXki6BSdqqcOfzO1g',
              }
    while count<30:
        try:
            count += 1
            #print '[+] Loading...'
            result = requests.get('%s%s'%(res_url, token), headers=headers)
            status = result.json()['status']
            if status == 'completed':
                #print '[+] Pic: %s' % url
                #print '[*] Name: %s' % result.json()[u'name']
                print '%s' % result.json()[u'name']
                break
        except Exception, e:
            print '[-] False: %s' % str(e)
            pass


if __name__ == '__main__':
    try:
        url = sys.argv[1]
        _api(url)
    except Exception, e:
        print 'Usage: cloudsightapi.py http://pic_url.com/test.jpg'
        print 'Error: %s' % str(e)
        pass
