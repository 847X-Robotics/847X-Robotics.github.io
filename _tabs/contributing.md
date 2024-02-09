---
# the default layout is 'page'
icon: fas fa-info-circle
order: 5
---
Contributing is quick and simple!

# Upload Format
First, go to the folder `_data` and in that folder, there should be the file `authors.yml`

## Creating An Author Object
At the bottom of `authors.yml`, create a new new variable for your Author name. You only need to do this once. 
```md
{author_id}:
  name: {full name}
```
The `author_id` must be unique and can not be the same as another ID in the `authors.yml` file. Your `full name` on the other hand can be whatever you want and is what will be shown when the `author_id` is typed.
### Setting author url

Starting from August 6, 2021  [Google recommends](https://developers.google.com/search/updates)  to set the  `author.url`  property. This property helps Google to disambiguate the correct author.

You can set it the same way as the other author properties. You can put it in an  `author`  object.

```md
{author_id}:
   name: {full name}
   url: {homepage_of_author}
```
### Twitter URL
If you choose to, you can also add a twitter URL to your `author` object like so.
```md
{author_id}:
   name: {full name}
   twitter: {twitter_of_author}
   url: {homepage_of_author}
```
### Picture
A picture can be added with this code.
```md
picture: /img/example.png
```
### Example:
```md
Prahas:
  name: Prahas Duggireddy 
  url: https://github.com/Raptorly1

Tropix:
  name: Tropix Official
  url: https://github.com/Tropix126
```

## Creating A Page

## Naming and Path

Generate a new file with the name `YYYY-MM-DD-TITLE.EXTENSION` and put it in the `_posts` directory located in the root folder. Ensure that the `EXTENSION` is either `md` or `markdown`.

## Front Matter
At the top of your page, you must specify a few things.
```md
---
title: TITLE
author: <author_id> # your unique author ID
date: YYYY-MM-DD HH:MM:SS +/-TTTT
categories: [TOP_CATEGORIE, SUB_CATEGORIE]
tags: [TAG]     # TAG names should always be lowercase
---
```
### author:
```
---  author:  <author_id>  # for single entry  
# or  
authors:  [<author1_id>,  <author2_id>]  # for multiple entries  
---
```
### title
The title can be whatever you want.
#### Example:
`title: Odometry Explanation`
### date:
The `+/-TTTT` is your timezone and in UTC offset and should be written after the date. You can use [this website](https://www.timeanddate.com/time/map/) to help find your UTC offset.  
### categories
A categorie creates a folder for pages that are similar.  You can have 1-2 categories.

You can put anything you want but please check existing categories first and try to make your categories prexisting ones for organization purposes.
### tags
You can list 0 - infinity tags..

Like categories, please try to make the tags preixisting ones. Make sure tags are lowercase

## Advanced Frontmatter

### Mathematics

For website performance reasons, the mathematical feature won’t be loaded by default. But it can be enabled by:
```
---
math: true
---
```
### Image
A thumbnail for your page can be added like this
```
image:
  path: https://i.postimg.cc/grFzjG4w/Tropical-pfp.png
  ```
 #### Dimensions
 You can change the dimensions by adding the `height` and `width` in pixels.
 ```
image:
  path: https://i.postimg.cc/grFzjG4w/Tropical-pfp.png
  width: 640 #correct image size
  height: 480
  ```
#### Caption

Add italics to the next line of an image, then it will become the caption and appear at the bottom of the image:

`![img-description](/path/to/image)`
_Image Caption_